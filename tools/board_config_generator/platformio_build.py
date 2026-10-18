"""Generate board headers and verify the selected MCU before a firmware build."""
import importlib.util
from pathlib import Path

Import("env")

root = Path(env.subst("$PROJECT_DIR"))
spec = importlib.util.spec_from_file_location("board_config_generator", root / "tools/board_config_generator/generator.py")
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)
boards = generator.load_boards(root / "boards")
flags = env.ParseFlags(env.GetProjectOption("build_flags", ""))
defines = {item[0] if isinstance(item, (tuple, list)) else item for item in flags.get("CPPDEFINES", [])}
board = generator.select_build_board(boards, defines, env.BoardConfig().get("build.mcu", ""))
generator.generate(root / "boards", root / "include")
print(f"Board configuration: {board['board']['revision']} ({board['mcu']['part']})")
