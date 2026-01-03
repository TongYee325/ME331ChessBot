import sys
import chess
import chess.engine
import chess.svg
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QMessageBox, QVBoxLayout
from PyQt5.QtSvg import QSvgWidget
from PyQt5.QtCore import Qt

# ================= 配置区域 =================
STOCKFISH_PATH = "/usr/games/stockfish"  # 请确保路径正确
BOARD_SIZE = 600

class ChessWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Chess Interface (Final Position Only)")
        self.setGeometry(100, 100, BOARD_SIZE, BOARD_SIZE + 50)

        self.board = chess.Board()
        
        # 1. 初始化 ID 映射表 (URDF 文件名)
        self.piece_id_map = {} 
        self.init_piece_ids()

        try:
            self.engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)
        except FileNotFoundError:
            print(f"错误: 找不到 Stockfish，请检查路径: {STOCKFISH_PATH}")
            sys.exit(1)

        self.layout = QVBoxLayout()
        self.svg_widget = QSvgWidget()
        self.svg_widget.setFixedSize(BOARD_SIZE, BOARD_SIZE)
        self.layout.addWidget(self.svg_widget)
        
        self.status_label = QLabel("你的回合 (白棋)")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.status_label)
        
        self.setLayout(self.layout)
        self.selected_square = None
        self.update_board_visual()

    def init_piece_ids(self):
        """初始化 ID (顺时针/逆时针排序)"""
        # === PION (兵) ===
        # 白兵 (Rank 2): h2(pion_1) -> a2(pion_8)
        for i in range(8):
            sq = chess.square(7 - i, 1) 
            self.piece_id_map[sq] = f"pion_{i+1}"

        # 黑兵 (Rank 7): a7(pion_9) -> h7(pion_16)
        for i in range(8):
            sq = chess.square(i, 6)
            self.piece_id_map[sq] = f"pion_{8 + i + 1}"

        # === BENTENG (车) ===
        self.piece_id_map[chess.H1] = "benteng_1"
        self.piece_id_map[chess.A1] = "benteng_2"
        self.piece_id_map[chess.A8] = "benteng_3"
        self.piece_id_map[chess.H8] = "benteng_4"

        # === JARAN (马) ===
        self.piece_id_map[chess.G1] = "jaran_1"
        self.piece_id_map[chess.B1] = "jaran_2"
        self.piece_id_map[chess.B8] = "jaran_3"
        self.piece_id_map[chess.G8] = "jaran_4"

        # === PATIH (象) ===
        self.piece_id_map[chess.F1] = "patih_1"
        self.piece_id_map[chess.C1] = "patih_2"
        self.piece_id_map[chess.C8] = "patih_3"
        self.piece_id_map[chess.F8] = "patih_4"

        # === STER (后) ===
        self.piece_id_map[chess.D1] = "ster_1"
        self.piece_id_map[chess.D8] = "ster_2"

        # === RAJA (王) ===
        self.piece_id_map[chess.E1] = "raja_1"
        self.piece_id_map[chess.E8] = "raja_2"

    def update_piece_ids(self, move):
        """移动后更新 ID"""
        src = move.from_square
        dst = move.to_square
        
        moving_id = self.piece_id_map.get(src, "Unknown_ID")
        
        # 1. 王车易位处理
        if self.board.is_castling(move):
            if move.to_square > move.from_square: # 王侧
                rook_src, rook_dst = move.to_square + 1, move.to_square - 1
            else: # 后侧
                rook_src, rook_dst = move.to_square - 2, move.to_square + 1
            
            if rook_src in self.piece_id_map:
                self.piece_id_map[rook_dst] = self.piece_id_map[rook_src]
                del self.piece_id_map[rook_src]

        # 2. 吃过路兵处理
        if self.board.is_en_passant(move):
            captured_square = dst - 8 if self.board.turn == chess.WHITE else dst + 8
            if captured_square in self.piece_id_map:
                del self.piece_id_map[captured_square]

        # 3. 普通移动
        self.piece_id_map[dst] = moving_id
        if src in self.piece_id_map:
            del self.piece_id_map[src]

    def process_move_sequence(self, move, is_human):
        """处理流程：输出数据 -> 移动逻辑 -> 刷新界面"""
        # 获取当前移动棋子的 ID
        current_id = self.piece_id_map.get(move.from_square, "Unknown")
        target_name = chess.square_name(move.to_square)
        source_name = chess.square_name(move.from_square) # 仅用于人类阅读的Log
        player_str = "[玩家]" if is_human else "[AI引擎]"

        # ==========================================
        # 修改后的输出格式：ID, 目标位置
        # ==========================================
        print(f"{player_str} 移动: {current_id} ({source_name} -> {target_name})")
        
        # 你的目标格式: "DATA_OUTPUT: pion_12, d4"
        print(f"DATA_OUTPUT: {current_id}, {target_name}")
        print("-" * 40)

        # 必须先更新 ID，再执行逻辑移动
        self.update_piece_ids(move)
        self.board.push(move)
        self.update_board_visual()

    def mousePressEvent(self, event):
        if self.board.turn == chess.BLACK: return

        click_x = event.x() - self.svg_widget.x()
        click_y = event.y() - self.svg_widget.y()
        square_size = BOARD_SIZE / 8
        file_idx = int(click_x // square_size)
        rank_idx = 7 - int(click_y // square_size)
        
        if 0 <= file_idx <= 7 and 0 <= rank_idx <= 7:
            clicked_square = chess.square(file_idx, rank_idx)
            
            if self.selected_square is None:
                piece = self.board.piece_at(clicked_square)
                if piece and piece.color == chess.WHITE:
                    self.selected_square = clicked_square
                    piece_id = self.piece_id_map.get(clicked_square, "Unknown")
                    self.status_label.setText(f"选中: {piece_id}")
            else:
                move = chess.Move(self.selected_square, clicked_square)
                if self.board.piece_at(self.selected_square).piece_type == chess.PAWN:
                     if (chess.square_rank(clicked_square) == 7):
                         move.promotion = chess.QUEEN

                if move in self.board.legal_moves:
                    self.process_move_sequence(move, is_human=True)
                    self.selected_square = None
                    self.status_label.setText("AI 思考中...")
                    QApplication.processEvents()
                    self.ai_move()
                else:
                    self.status_label.setText("非法移动")
                    self.selected_square = None

    def ai_move(self):
        if self.board.is_game_over():
            self.game_over()
            return

        result = self.engine.play(self.board, chess.engine.Limit(time=0.5))
        if result.move:
            self.process_move_sequence(result.move, is_human=False)
            self.status_label.setText("你的回合 (白棋)")
            if self.board.is_game_over():
                self.game_over()

    def update_board_visual(self):
        svg_bytes = chess.svg.board(self.board, size=BOARD_SIZE).encode("UTF-8")
        self.svg_widget.load(svg_bytes)

    def game_over(self):
        QMessageBox.information(self, "Game Over", f"结果: {self.board.result()}")

    def closeEvent(self, event):
        self.engine.quit()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ChessWindow()
    window.show()
    sys.exit(app.exec_())