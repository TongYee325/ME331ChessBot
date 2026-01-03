import sys
import chess
import chess.svg
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QMessageBox, QVBoxLayout
from PyQt5.QtSvg import QSvgWidget
from PyQt5.QtCore import Qt
import rospy
from std_msgs.msg import String

# ================= 配置区域 =================
BOARD_SIZE = 600

# ================= 坐标映射配置 =================
# Gazebo 坐标系参数 (单位: 米)
# a8 (Col 0, Rank 7)
POS_A8_X = 0.148265
POS_A8_Y = -0.152139
# h1 (Col 7, Rank 0)
POS_H1_X = 0.451109
POS_H1_Y = 0.151845
FIXED_Z = 0.022

class ChessWindow(QWidget):
    def __init__(self):
        super().__init__()
        
        # ROS 初始化
        rospy.init_node('chess_gui_node', anonymous=True)
        self.move_pub = rospy.Publisher('/chess_move', String, queue_size=10)

        self.setWindowTitle("Robot Chess Interface (Local PvP)")
        self.setGeometry(100, 100, BOARD_SIZE, BOARD_SIZE + 50)

        self.board = chess.Board()
        
        # 1. 初始化 ID 映射表
        self.piece_id_map = {} 
        self.init_piece_ids()

        self.layout = QVBoxLayout()
        self.svg_widget = QSvgWidget()
        self.svg_widget.setFixedSize(BOARD_SIZE, BOARD_SIZE)
        self.layout.addWidget(self.svg_widget)
        
        self.status_label = QLabel("白棋回合")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.status_label)
        
        self.setLayout(self.layout)
        self.selected_square = None
        self.update_board_visual()

    def get_gazebo_coords(self, square_idx):
        """
        将棋盘格子索引转换为 Gazebo (x, y, z) 坐标
        """
        # 获取列 (File 0-7) 和 行 (Rank 0-7)
        file_idx = chess.square_file(square_idx) # a=0, h=7
        rank_idx = chess.square_rank(square_idx) # 1=0, 8=7

        # 线性插值计算
        # X 轴由 Rank 决定 (从 Rank 1 到 Rank 8)
        # 这里的公式基于 h1(rank 0) 到 a8(rank 7)
        # 计算每一步 Rank 的变化量
        total_delta_x = POS_A8_X - POS_H1_X
        step_x = total_delta_x / 7.0
        target_x = POS_H1_X + (rank_idx * step_x)

        # Y 轴由 File 决定 (从 a 到 h)
        # a8(file 0) 到 h1(file 7)
        total_delta_y = POS_H1_Y - POS_A8_Y
        step_y = total_delta_y / 7.0
        target_y = POS_A8_Y + (file_idx * step_y)

        return target_x, target_y, FIXED_Z

    def init_piece_ids(self):
        """初始化 ID (URDF 命名规则)"""
        # === PION (兵) ===
        for i in range(8):
            sq = chess.square(7 - i, 1) 
            self.piece_id_map[sq] = f"pion{i+1}"
        for i in range(8):
            sq = chess.square(i, 6)
            self.piece_id_map[sq] = f"pion{8 + i + 1}"

        # === 其他棋子 ===
        self.piece_id_map[chess.H1] = "benteng1"
        self.piece_id_map[chess.A1] = "benteng2"
        self.piece_id_map[chess.A8] = "benteng3"
        self.piece_id_map[chess.H8] = "benteng4"

        self.piece_id_map[chess.G1] = "jaran1"
        self.piece_id_map[chess.B1] = "jaran2"
        self.piece_id_map[chess.B8] = "jaran3"
        self.piece_id_map[chess.G8] = "jaran4"

        self.piece_id_map[chess.F1] = "patih1"
        self.piece_id_map[chess.C1] = "patih2"
        self.piece_id_map[chess.C8] = "patih3"
        self.piece_id_map[chess.F8] = "patih4"

        self.piece_id_map[chess.D1] = "ster1"
        self.piece_id_map[chess.D8] = "ster2"

        self.piece_id_map[chess.E1] = "raja1"
        self.piece_id_map[chess.E8] = "raja2"
    def update_piece_ids(self, move):
        src, dst = move.from_square, move.to_square
        moving_id = self.piece_id_map.get(src, "Unknown_ID")
        
        # 特殊规则处理
        if self.board.is_castling(move):
            if move.to_square > move.from_square:
                r_src, r_dst = move.to_square + 1, move.to_square - 1
            else:
                r_src, r_dst = move.to_square - 2, move.to_square + 1
            if r_src in self.piece_id_map:
                self.piece_id_map[r_dst] = self.piece_id_map[r_src]
                del self.piece_id_map[r_src]

        if self.board.is_en_passant(move):
            cap_sq = dst - 8 if self.board.turn == chess.WHITE else dst + 8
            if cap_sq in self.piece_id_map: del self.piece_id_map[cap_sq]

        self.piece_id_map[dst] = moving_id
        if src in self.piece_id_map: del self.piece_id_map[src]

    def process_move_sequence(self, move):
        current_id = self.piece_id_map.get(move.from_square, "Unknown")
        target_name = chess.square_name(move.to_square)
        
        player_name = "白棋" if self.board.turn == chess.WHITE else "黑棋"
        player = f"[{player_name}]"

        # === 新增：检测并处理吃子 ===
        if self.board.is_capture(move):
            # 确定被吃掉的棋子位置 (处理过路兵 En Passant)
            if self.board.is_en_passant(move):
                captured_sq = move.to_square - 8 if self.board.turn == chess.WHITE else move.to_square + 8
            else:
                captured_sq = move.to_square
            
            captured_id = self.piece_id_map.get(captured_sq)
            if captured_id:
                # 获取被吃棋子的物理坐标
                cx, cy, cz = self.get_gazebo_coords(captured_sq)
                # 计算弃子区坐标 (y + 0.5)
                discard_y = cy + 0.5
                
                print(f"检测到吃子: {captured_id} -> 弃子区 (y={discard_y:.4f})")
                
                # 发送移除指令
                msg_str_capture = f"{captured_id} {cx} {discard_y} {cz}"
                self.move_pub.publish(msg_str_capture)
                rospy.loginfo(f"Published capture move: {msg_str_capture}")
                
                # 稍微延时，确保消息顺序进入队列
                rospy.sleep(0.2)

        # === 核心修改：计算物理坐标 ===
        gx, gy, gz = self.get_gazebo_coords(move.to_square)
        
        # 格式化坐标字符串 (保留4位小数)
        coord_str = f"{gx:.4f}, {gy:.4f}, {gz:.3f}"

        print(f"{player} 移动: {current_id} -> {target_name}")
        
        # DATA_OUTPUT 格式: ID, 目标格名, X, Y, Z
        # 例如: DATA_OUTPUT: pion_12, d4, 0.3215, 0.0001, 0.022
        print(f"DATA_OUTPUT: {current_id}, {coord_str}")
        
        # 发送 ROS 指令
        # 格式: "piece_id x y z"
        msg_str = f"{current_id} {gx} {gy} {gz}"
        self.move_pub.publish(msg_str)
        rospy.loginfo(f"Published move: {msg_str}")

        print("-" * 50)

        self.update_piece_ids(move)
        self.board.push(move)
        self.update_board_visual()

    def mousePressEvent(self, event):
        click_x = event.x() - self.svg_widget.x()
        click_y = event.y() - self.svg_widget.y()
        square_size = BOARD_SIZE / 8
        file_idx = int(click_x // square_size)
        rank_idx = 7 - int(click_y // square_size)
        
        if 0 <= file_idx <= 7 and 0 <= rank_idx <= 7:
            clicked_square = chess.square(file_idx, rank_idx)
            if self.selected_square is None:
                piece = self.board.piece_at(clicked_square)
                if piece and piece.color == self.board.turn:
                    self.selected_square = clicked_square
                    piece_id = self.piece_id_map.get(clicked_square, "Unknown")
                    self.status_label.setText(f"选中: {piece_id}")
            else:
                move = chess.Move(self.selected_square, clicked_square)
                if self.board.piece_at(self.selected_square).piece_type == chess.PAWN:
                     if (chess.square_rank(clicked_square) == 7 or chess.square_rank(clicked_square) == 0): 
                         move.promotion = chess.QUEEN
                if move in self.board.legal_moves:
                    self.process_move_sequence(move)
                    self.selected_square = None
                    
                    next_player = "白棋" if self.board.turn == chess.WHITE else "黑棋"
                    self.status_label.setText(f"{next_player}回合")
                    
                    if self.board.is_game_over(): self.game_over()
                else:
                    self.status_label.setText("非法移动")
                    self.selected_square = None

    def update_board_visual(self):
        self.svg_widget.load(chess.svg.board(self.board, size=BOARD_SIZE).encode("UTF-8"))
    def game_over(self):
        QMessageBox.information(self, "Game Over", f"结果: {self.board.result()}")
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ChessWindow()
    window.show()
    sys.exit(app.exec_())