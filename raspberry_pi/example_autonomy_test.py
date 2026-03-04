#!/usr/bin/env python3
"""
自律飛行状態管理のテストサンプル
実際のフライトコントローラーなしでモック動作を確認
"""

import time
from autonomy_state import AutonomyStateManager, FlightControllerState, RaspberryPiState


class MockMAVLink:
    """モックMAVLink接続"""

    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.sent_commands = []
        self.test_mode = 0
        self.test_state = 0

    def recv_match(self, type=None, blocking=False):
        """モックメッセージ受信"""
        if self.test_mode == 99:
            # テスト用のCOMMAND_LONGメッセージを返す
            class MockMessage:
                def __init__(self, mode, state):
                    self.param1 = mode
                    self.param2 = state

                def get_type(self):
                    return 'COMMAND_LONG'

            return MockMessage(self.test_mode, self.test_state)
        return None

    class mav:
        @staticmethod
        def command_long_send(*args, **kwargs):
            """モックコマンド送信"""
            print(f"  [MAVLink] RPI状態送信: {args[2]}, param1={args[3]}")


class MockFlightController:
    """モックフライトコントローラー"""

    def __init__(self):
        self.current_position = [0.0, 0.0, -50.0]
        self.waypoints = []
        self.current_wp_index = 0

    def set_waypoints(self, waypoints):
        """モックウェイポイント設定"""
        print(f"  [FC] ウェイポイント設定: {len(waypoints)}個")
        self.waypoints = list(waypoints)
        return True


def simulate_mission():
    """ミッション全体の流れをシミュレーション"""

    print("=== 自律飛行状態管理テスト ===\n")

    # モックオブジェクト作成
    mavlink = MockMAVLink()
    flight = MockFlightController()
    autonomy = AutonomyStateManager(mavlink, flight)

    print("初期状態:")
    print(f"  RPI: {autonomy.rpi_state.name}")
    print(f"  FC: {autonomy.fc_state.name}\n")

    # ステップ1: PLANNING
    print("--- ステップ1: PLANNING ---")
    print("FC → RPI: Mode=99, State=PLANNING")
    mavlink.test_mode = 99
    mavlink.test_state = FlightControllerState.PLANNING.value
    autonomy.update_state()
    time.sleep(0.1)

    print(f"RPI状態: {autonomy.rpi_state.name}")
    print(f"ウェイポイント数: {len(autonomy.waypoints)}\n")
    time.sleep(1)

    # ステップ2: INITIALIZING
    print("--- ステップ2: INITIALIZING ---")
    print("FC → RPI: Mode=99, State=INITIALIZING")
    mavlink.test_state = FlightControllerState.INITIALIZING.value
    autonomy.update_state()
    time.sleep(0.1)

    print(f"RPI状態: {autonomy.rpi_state.name}\n")
    time.sleep(1)

    # ステップ3: EXECUTING (開始)
    print("--- ステップ3: EXECUTING (開始) ---")
    print("FC → RPI: Mode=99, State=EXECUTING")
    mavlink.test_state = FlightControllerState.EXECUTING.value
    autonomy.update_state()
    time.sleep(0.1)

    print(f"RPI状態: {autonomy.rpi_state.name}")
    print(f"ミッション開始: {autonomy.mission_started}\n")
    time.sleep(1)

    # ステップ4: EXECUTING (継続)
    print("--- ステップ4: EXECUTING (継続) ---")
    for i in range(5):
        print(f"  飛行中... ({i+1}/5)")
        autonomy.update_state()
        time.sleep(0.5)

    # 全ウェイポイント到達をシミュレート
    flight.current_wp_index = len(autonomy.waypoints)
    autonomy.update_state()
    print(f"RPI状態: {autonomy.rpi_state.name}\n")
    time.sleep(1)

    # ステップ5: IDLE
    print("--- ステップ5: IDLE ---")
    print("FC → RPI: Mode=99, State=IDLE")
    mavlink.test_state = FlightControllerState.IDLE.value
    autonomy.update_state()
    time.sleep(0.1)

    print(f"RPI状態: {autonomy.rpi_state.name}")
    print(f"ミッション開始: {autonomy.mission_started}")
    print(f"ウェイポイント数: {len(autonomy.waypoints)}\n")

    # 状態情報表示
    print("--- 最終状態情報 ---")
    state_info = autonomy.get_state_info()
    for key, value in state_info.items():
        print(f"  {key}: {value}")

    print("\n=== テスト完了 ===")


if __name__ == "__main__":
    simulate_mission()
