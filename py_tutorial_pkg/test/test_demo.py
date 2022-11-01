from py_tutorial_pkg import demo

def test_math():
    result = demo.func_under_test(2, 2)
    assert result == 4