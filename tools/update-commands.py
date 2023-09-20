import subprocess

subprocess.call(["python", "./commands_generator/generator.py", "./../commands.json", "./../include/commands.h", "--language", "c"])
