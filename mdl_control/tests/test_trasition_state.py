#!/usr/bin/env python3

from scripts.assembly import Assembly
from scripts.trasition_state import TransitionState

import numpy as np
import numpy.typing as npt

import pytest


def test_get_able_eliminate_modules_id():
    """
    必要な機能
    - get_able_eliminate_modules_id
        - 外側のモジュールのIDを取得する: get_frontier_module_ids
        - 削除したときのネットワークの連結性を確認する: check_connectivity
    """
    asm_first = Assembly(num_cubes=27)
    asm_first.create_cubic_assembly()
    state = TransitionState(asm_first)
    assert np.array_equal(state.asm.robot_base_pos, np.array([0, 0, 0]))
    assert state.get_able_eliminate_modules_id() == [
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15, 16, 17,
        18, 19, 20, 21, 22, 23, 24, 25, 26]


def test_get_able_add_modules_pos():
    """
    必要な機能
    - get_able_add_modules_pos
        - 最も外側のモジュールの外側の位置を取得する: get_near_asm_pos
        - 位置が重なっているか確認する: is_pos_free
    - update_graph
    """
    asm_first = Assembly(num_cubes=27)
    asm_first.create_cubic_assembly()
    state = TransitionState(asm_first)
    id_list_eliminate_modules = state.get_able_eliminate_modules_id()
    id_selected = id_list_eliminate_modules[1]
    assert id_selected == 1
    asm_eliminate = state.eliminate_module(id_selected)
    assert np.array_equal(asm_eliminate.robot_base_pos,
                          np.array([0, 0, 0.12]))
    pos_list_add: list[npt.NDArray] = state.get_able_add_modules_pos(
        asm_eliminate, id_selected)
    assert len(pos_list_add) == 56
