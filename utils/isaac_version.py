import re

def _detect():
    try:
        from importlib.metadata import version
        raw = version("isaacsim")
    except Exception:
        import omni.kit.app
        raw = omni.kit.app.get_app().get_build_version()
    nums = re.findall(r"\d+", raw)[:3]
    return tuple(int(n) for n in nums) or (0,)

VERSION = _detect()
IS_6 = VERSION[0] >= 6
