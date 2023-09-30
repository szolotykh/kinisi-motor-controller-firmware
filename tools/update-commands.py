import subprocess

# Update command file
subprocess.call(["python", "./commands_generator/generator.py", "./../commands.json", "./../include/commands.h", "--language", "c"])

# Update documentation
subprocess.call(["python", "./commands_generator/generator.py", "./../commands.json", "./../commands.md", "--language", "md"])