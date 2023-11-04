import argparse
import datetime
import json
import os

# Map of type names to their size in bytes
type_to_size_map = {
    "uint8_t": 1,
    "uint16_t": 2,
    "uint32_t": 4,
    "int8_t": 1,
    "int16_t": 2,
    "int32_t": 4,
    "double": 8,
    "integer": 4,
}

def file_header(version, comment_char="//"):
    return f"""{comment_char} ----------------------------------------------------------------------------
{comment_char} Kinisi motr controller commands.
{comment_char} This file is auto generated by the commands generator from JSON file.
{comment_char} Do not edit this file manually.
{comment_char} Timestamp: { datetime.datetime.now().strftime("%Y-%m-%d %H:%M:00") }
{comment_char} Version: {version}
{comment_char} ----------------------------------------------------------------------------\n\n"""

# Function to validate names for various categories like 'Type', 'Command', etc.
# 'name' is the name to be validated, 'category' specifies the category, and 'context' provides additional information.
def validate_name(name, category, context):
    errors = []
    # Check if the name contains a space
    if " " in name:
        errors.append(f"{category} name '{name}' in {context} contains a space.")
    return errors


# Function to validate a command in the JSON
# 'command' is a dictionary containing the command details
def validate_command(command):
    errors = []
    # Validate the name of the command
    errors.extend(validate_name(command['command'], 'Command', 'commands'))
    # Validate the names and ranges of properties for the command
    for prop in command.get("properties", []):
        errors.extend(validate_name(prop['name'], 'Property', f"command '{command['command']}'"))
        # Check if a range exists and if so, validate it
        if "range" in prop:
            if not isinstance(prop["range"], list) or len(prop["range"]) != 2 or prop["range"][0] > prop["range"][1]:
                errors.append(f"Invalid range '{prop['range']}' in property '{prop['name']}' for command '{command['command']}'.")
    return errors

# Main function to validate the entire JSON
# 'commands_data' is a dictionary containing the JSON data
def validate_json(commands_data):
    errors = []

    # Validate each command in the JSON
    for command in commands_data.get("commands", []):
        errors.extend(validate_command(command))
    return errors

# Generates JavaScript code from the commands JSON file
def generate_js_code(commands_data):

    # Map of type names to their corresponding setter functions in the DataView class
    type_to_js_setter_func_map = {
        "uint8_t": {
            "set":"setUint8",
            "get":"getUint8"
            },
        "uint16_t": {
            "set":"setUint16",
            "get":"getUint16"
            },
        "uint32_t": {
            "set":"setUint32",
            "get":"getUint32"
            },
        "int8_t": {
            "set":"setInt8", 
            "get":"getInt8"
            },
        'int16_t': {
            "set":"setInt16",
            "get":"getInt16"
            },
        'int32_t': {
            "set":"setInt32",
            "get":"getInt32"
            },
        "double": {
            "set":"setFloat64",
            "get":"getFloat64"
            },
    }
    
    # Generate constants for command codes
    constant_code = ""
    for cmd in commands_data['commands']:
        constant_code += f"const {cmd['command']} = {cmd['code']}\n"
    constant_code += "\n"

    # Write abstract methods for write and read
    abstract_methods = "    async write(buffer) {\n"
    abstract_methods += "        throw new Error(\"write method must be implemented\");\n"
    abstract_methods += "    }\n\n"

    abstract_methods += "    async read(numBytes) {\n"
    abstract_methods += "        throw new Error(\"read method must be implemented\");\n"
    abstract_methods += "    }\n\n"

    # Generate function for each command
    function_code = ""
    for cmd in commands_data['commands']:
        func_args = ', '.join([f"{prop['name']}" for prop in cmd.get('properties', [])])
        func_body = f"    async {cmd['command'].lower()}({func_args}){{\n"

        message_length = 1  # 1 byte for the command code
        for prop in cmd.get("properties", []):
            message_length += type_to_size_map.get(prop['type'], 1)  # Default to 1 byte if type is unknown

        func_body += f"        const messageLength = {message_length};\n"
        func_body += f"        const buffer = new ArrayBuffer(messageLength + 1);\n"
        func_body += f"        const view = new DataView(buffer);\n"

        func_body += f"        view.setUint8(0, messageLength);  // Message length\n"
        func_body += f"        view.setUint8(1, {cmd['code']});  // Command byte\n"

        offset = 2
        for prop in cmd.get("properties", []):
            setter_func = type_to_js_setter_func_map[prop['type']].get("set")
            little_endian = ", true" if type_to_size_map[prop['type']] > 1 else ""
            func_body += f"        view.{setter_func}({offset}, {prop['name']}{little_endian});  // {prop['name']}\n"
            offset += type_to_size_map.get(prop['type'], 1)

        func_body += "        await this.write(buffer);\n"

        response = cmd.get("response", None)
        if response != None:
            func_body += f"        const result = await this.read({type_to_size_map[response['type']]})\n"
            func_body += f"        const dataView = new DataView(result, 0);\n"
            # if type size map is 1, then it is a single byte and we don't need to specify the endianness
            if type_to_size_map[response['type']] == 1:
                func_body += f"        return dataView.{type_to_js_setter_func_map[response['type']].get('get')}(0);\n"
            else:
                func_body += f"        return dataView.{type_to_js_setter_func_map[response['type']].get('get')}(0, true);\n"

        func_body += "    }\n\n"
        function_code += func_body

    # Generate the final code
    result = file_header(commands_data['version'])
    result += constant_code
    result += "class Commands {\n"
    result += abstract_methods
    result += function_code
    result += "}"

    return result

# Generates Python code from the commands JSON file
def generate_python_code(commands_data):

    type_mapping = {
        'uint8_t': 'int',
        'uint16_t': 'int',
        'uint32_t': 'int',
        'double': 'float',
        'int8_t': 'int',
        'int16_t': 'int',
        'int32_t': 'int',
    }

    # Initialize code strings
    constant_code = ''
    function_code = ''
    class_code = '''
class KinisiCommands:
    # Write command to serial interface
    def write(self, msg: bytearray):
        """Abstract method to write the byte message to the serial interface."""
        raise NotImplementedError("This method should be overridden by subclass.")
    
    # Read command from serial interface
    def read(self, length: int) -> bytearray:
        """Abstract method to read a specified number of bytes from the serial interface."""
        raise NotImplementedError("This method should be overridden by subclass.")

    '''
    
    # Generate constants for command codes
    for cmd in commands_data['commands']:
        constant_code += f"{cmd['command']} = {cmd['code']}\n"
    constant_code += "\n"


    # Generate function for each command
    for cmd in commands_data['commands']:
        func_body = f"    # {cmd['description']}\n"
        func_args = ", ".join([f"{prop['name']}:{type_mapping[prop['type']]}" for prop in cmd.get('properties', [])])
        func_body += f"    def {cmd['command'].lower()}(self{', ' if len(func_args) > 0 else ''}{func_args}):\n"
        func_body += f"        msg = {cmd['command']}.to_bytes(1, 'little')"
        for prop in cmd.get('properties', []):
            if prop['type'] == 'double':
                func_body += f" + bytearray(struct.pack('d', {prop['name']}))"
            else:
                signed = ", signed=True" if prop['type'].startswith("int") else ""
                func_body += f" + {prop['name']}.to_bytes({type_to_size_map[prop['type']]}, 'little'{signed})"

        func_body += "\n"
        func_body += f"        length = len(msg)\n"
        func_body += f"        msg = length.to_bytes(1, 'little') + msg\n"
        func_body += f"        self.write(msg)\n"
        if 'response' in cmd:
            func_body += f"        result =  self.read({type_to_size_map[cmd['response']['type']]})\n"
            if cmd['response']['type'] == 'double':
                func_body += f"        return struct.unpack('<d', result)[0]\n"
            else:
                func_body += f"        return {type_mapping[cmd['response']['type']]}.from_bytes(result, 'little')\n"
        function_code += func_body
        function_code += "\n"

    class_code += function_code

    result = file_header(commands_data['version'], "#")
    result += "import struct\n\n"
    result += constant_code
    result += class_code

    return result

# Generates a Markdown file from the commands JSON file
def generate_md_file(commands_data):
    md_content = "# Kinisi motor controller commands\n\n"

    # version
    md_content += f"Version: {commands_data['version']}\n"
    md_content += "---\n"

    # Generate command definitions
    md_content += "\n## Commands\n"
    for command in commands_data.get("commands", []):
        md_content += f"### {command['command']} ({command['code']})\n"
        md_content += f"Description: {command['description']}\n"

        # Generate property definitions
        md_content += f"Properties:\n"
        cmd_props = command.get("properties", [])
        if len(cmd_props) != 0:
            for prop in cmd_props:
                md_content += f"- {prop['name']} ({prop.get('type', 'unknown')})"
                if "description" in prop:
                    md_content += f": {prop['description']}"
                md_content += "\n"
                if "range" in prop:
                    md_content += f"  - Range: {prop['range'][0]} to {prop['range'][1]}\n"
        else:
            md_content += "- None\n"

        # Generate response definition
        if "response" in command:
            response = command.get("response", [])
            md_content += f"Response: \n - {response['name']} ({response['type']}): {response['description']}\n"

        md_content += "\n"
    return md_content


# Generates a C header file from the commands JSON file
def generate_c_header_file(commands_data):
    # Create commands definitions
    definitions = "// Commands"
    for cmd in commands_data['commands']:
        definitions +=  f"// {cmd['description']}\n"
        definitions += f"#define {cmd['command']} {cmd['code']}\n\n"

    commands_struct = "#pragma pack(push, 1)\n"
    commands_struct += "typedef struct\n"
    commands_struct += "{\n"
    commands_struct += "    uint8_t commandType;  //command type\n"
    commands_struct += "    union {   //Union to support different properties of each command\n"

    for cmd in commands_data['commands']:
        commands_struct += f"        // {cmd['command']}: {cmd['description']}\n"
        commands_struct += "        struct {\n"
        for prop in cmd.get("properties", []):
            commands_struct += f"            {prop['type']} {prop['name']}; // {prop['description']}\n"
        commands_struct += f"        }} {cmd['command'].lower()};\n\n"

    commands_struct += "    } properties;\n"
    commands_struct += "} controller_command_t;\n"
    commands_struct += "#pragma pack(pop)\n\n"
    
    result = file_header(commands_data['version'])
    result += "#pragma once\n\n"
    result += "#include <stdint.h>\n\n"
    result += definitions
    result += "\n"
    result += commands_struct

    return result

# Generates C++ code from the commands JSON file
def generate_cpp_code(commands_data):
    # Create commands definitions
    definitions = "// Commands"
    for cmd in commands_data['commands']:
        definitions +=  f"// {cmd['description']}\n"
        definitions += f"#define {cmd['command']} {cmd['code']}\n\n"

    commands_class = "// Commands class\n"
    commands_class += "class KinisiCommands {\n"
    commands_class += "public:\n"
    commands_class += "    virtual int read(unsigned int lenght) = 0;\n\n"
    commands_class += "    virtual int write(unsigned char* cmd, unsigned int size) = 0;\n\n"

    # Generate class method for each command
    commands_class += "public:\n"
    for cmd in commands_data['commands']:
        commands_class += f"    // {cmd['command']}: {cmd['description']}\n"
        func_args = ', '.join([f"{prop['type']} {prop['name']}" for prop in cmd.get('properties', [])])
        commands_class += f"    void {cmd['command'].lower()}({func_args}){{\n"
        commands_class += f"        unsigned char cmd[] = {{ 0, {cmd['command']}, {', '.join([prop['name'] for prop in cmd.get('properties', [])])}}};\n"
        commands_class += f"        cmd[0] = sizeof(cmd) - 1;\n"
        commands_class += f"        write(cmd, sizeof(cmd));\n"
        # Generate response 
        if "response" in cmd:
            response = cmd.get("response", [])
            commands_class += f"        return read(typeof({response['type']}));\n"
        commands_class += "    }\n"
    commands_class += "};\n\n"

    # Put everything together
    result = file_header(commands_data['version'])
    result += "#pragma once\n\n"
    result += "#include <stdint.h>\n\n"
    result += "\n"
    result += definitions
    result += "\n"
    result += commands_class

    return result

# Converts a string with underscores into PascalCase:
def to_pascal(s):
    return ''.join(word.capitalize() for word in s.split('_'))

def main():
    parser = argparse.ArgumentParser(description='Generate code from command definitions in JSON.')
    parser.add_argument('input_json_path', type=str, help='Path to the input JSON file containing command definitions.')
    parser.add_argument('output_path', type=str, help='Path to the output file.')
    parser.add_argument('--language', type=str, default='js', choices=['c','cpp', 'js', 'python', 'md'], help='Programming language for the generated code.')

    args = parser.parse_args()

    # Check if the input JSON file exists
    if not os.path.exists(args.input_json_path):
        print(f"Error: Input JSON file '{args.input_json_path}' not found.")
        return

    try:
        with open(args.input_json_path, 'r') as file:
            commands_data = json.load(file)
    except json.JSONDecodeError:
        print("Error: Could not decode the input JSON file.")
        return
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return

    validation_errors = validate_json(commands_data)
    if validation_errors:
        print("Validation errors found:")
        for error in validation_errors:
            print(f"- {error}")
        return

    generated_code = ""
    if args.language == 'c':
        generated_code = generate_c_header_file(commands_data)
    elif args.language == 'cpp':
        generated_code = generate_cpp_code(commands_data)
    elif args.language == 'js':
        generated_code = generate_js_code(commands_data)
    elif args.language == 'python':
        generated_code = generate_python_code(commands_data)
    elif args.language == 'md':
        generated_code = generate_md_file(commands_data)
    else:
        print("Error: Unsupported programming language.")
        return

    try:
        with open(args.output_path, 'w') as file:
            file.write(generated_code)
    except Exception as e:
        print(f"An error occurred while writing to the output file: {e}")
        return

    print(f"{args.language.upper()} code has been generated and saved to {args.output_path}")

if __name__ == "__main__":
    main()
