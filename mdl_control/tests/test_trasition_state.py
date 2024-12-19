#!/usr/bin/env python3

from scripts.trasition_state import TrasitionState
import pytest


def test_get_able_eliminate_modules_id():
    """
    必要な機能
    - get_able_eliminate_modules_id
        - 外側のモジュールのIDを取得する: get_frontier_module_ids
        - 削除したときのネットワークの連結性を確認する: check_connectivity
    """
    state = TrasitionState()
    assert state.get_able_eliminate_modules_id() == []


def test_get_able_add_modules_pos():
    """
    必要な機能
    - get_able_add_modules_pos
        - 最も外側のモジュールの外側の位置を取得する: get_near_asm_pos
        - 位置が重なっているか確認する: is_pos_free
    - update_graph
    """
    state = TrasitionState()
    assert state.get_able_add_modules_pos(None, 1) == []
