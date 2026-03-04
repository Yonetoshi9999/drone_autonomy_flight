#!/usr/bin/env python3
"""
統合検証スクリプト
main.pyの統合が正しいか検証
"""

import ast
import sys


def validate_main_integration():
    """main.pyの統合を検証"""

    print("=== main.py 統合検証 ===\n")

    # main.pyを読み込み
    with open('main.py', 'r', encoding='utf-8') as f:
        source = f.read()

    # ASTを解析
    try:
        tree = ast.parse(source)
    except SyntaxError as e:
        print(f"❌ 構文エラー: {e}")
        return False

    checks = {
        'autonomy_import': False,
        'autonomy_init': False,
        'state_loop_defined': False,
        'state_loop_async': False,
        'state_loop_in_tasks': False,
        'state_period_defined': False
    }

    # インポート確認
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            if node.module == 'autonomy_state':
                if any(alias.name == 'AutonomyStateManager' for alias in node.names):
                    checks['autonomy_import'] = True
                    print("✓ autonomy_state のインポート確認")

    # クラス定義を探す
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == 'AerialPhotographyDrone':
            # __init__メソッドを探す
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == '__init__':
                    # self.autonomy の初期化を探す
                    for stmt in ast.walk(item):
                        if isinstance(stmt, ast.Assign):
                            for target in stmt.targets:
                                if isinstance(target, ast.Attribute):
                                    if target.attr == 'autonomy':
                                        checks['autonomy_init'] = True
                                        print("✓ AutonomyStateManager の初期化確認")
                                    elif target.attr == 'state_period':
                                        checks['state_period_defined'] = True
                                        print("✓ state_period の定義確認")

            # state_management_loopメソッドを探す
            for item in node.body:
                if isinstance(item, ast.AsyncFunctionDef) and item.name == 'state_management_loop':
                    checks['state_loop_defined'] = True
                    checks['state_loop_async'] = True
                    print("✓ state_management_loop メソッド定義確認 (async)")

            # runメソッドを探す
            for item in node.body:
                if isinstance(item, ast.AsyncFunctionDef) and item.name == 'run':
                    # tasksリストを探す
                    for stmt in ast.walk(item):
                        if isinstance(stmt, ast.Assign):
                            for target in stmt.targets:
                                if isinstance(target, ast.Name) and target.id == 'tasks':
                                    # リストの要素を確認
                                    if isinstance(stmt.value, ast.List):
                                        for elt in stmt.value.elts:
                                            if isinstance(elt, ast.Call):
                                                if isinstance(elt.func, ast.Attribute):
                                                    if isinstance(elt.func.value, ast.Name):
                                                        if elt.func.value.id == 'asyncio':
                                                            # create_taskの引数を確認
                                                            if len(elt.args) > 0:
                                                                arg = elt.args[0]
                                                                if isinstance(arg, ast.Call):
                                                                    if isinstance(arg.func, ast.Attribute):
                                                                        if arg.func.attr == 'state_management_loop':
                                                                            checks['state_loop_in_tasks'] = True
                                                                            print("✓ state_management_loop がタスクリストに追加確認")

    # 結果サマリー
    print("\n--- 検証結果 ---")
    all_passed = all(checks.values())

    for check_name, passed in checks.items():
        status = "✓" if passed else "❌"
        print(f"{status} {check_name}: {'OK' if passed else 'NG'}")

    print("\n" + "="*50)
    if all_passed:
        print("✓ 統合検証: すべてのチェックに合格")
        return True
    else:
        print("❌ 統合検証: 一部のチェックに失敗")
        failed = [name for name, passed in checks.items() if not passed]
        print(f"失敗した項目: {', '.join(failed)}")
        return False


def validate_autonomy_module():
    """autonomy_state.pyモジュールを検証"""

    print("\n=== autonomy_state.py モジュール検証 ===\n")

    try:
        from autonomy_state import (
            AutonomyStateManager,
            FlightControllerState,
            RaspberryPiState,
            StateMessage
        )
        print("✓ すべてのクラスをインポート成功")

        # Enumの値を確認
        print("\n--- FlightControllerState ---")
        for state in FlightControllerState:
            print(f"  {state.name} = {state.value}")

        print("\n--- RaspberryPiState ---")
        for state in RaspberryPiState:
            print(f"  {state.name} = {state.value}")

        # 必須メソッドの存在確認
        required_methods = [
            'receive_state_from_fc',
            'send_state_to_fc',
            'update_state',
            'get_state_info',
            'is_autonomous_active'
        ]

        print("\n--- 必須メソッド確認 ---")
        for method in required_methods:
            if hasattr(AutonomyStateManager, method):
                print(f"✓ {method}")
            else:
                print(f"❌ {method} が見つかりません")
                return False

        return True

    except ImportError as e:
        print(f"❌ インポートエラー: {e}")
        return False


if __name__ == "__main__":
    result1 = validate_autonomy_module()
    result2 = validate_main_integration()

    if result1 and result2:
        print("\n" + "="*50)
        print("✓✓✓ すべての検証に合格 ✓✓✓")
        sys.exit(0)
    else:
        print("\n" + "="*50)
        print("❌ 検証失敗")
        sys.exit(1)
