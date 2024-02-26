# Kinisi Motor Controller Command Generator

This script is designed to generate command-related code for the Kinisi motor controller based on a JSON file containing command definitions. The produced code can be in various languages, such as JavaScript, Python, C, C++, and Markdown.

## How to Use:

### Requirements:

- Python 3.x

### Running the script:

Use the following command format:

```
python generator.py <path_to_input_json> <path_to_output_file> --language <target_language>
```
- `<path_to_input_json>`: Path to the JSON file containing command definitions.
- `<path_to_output_file>`: Path where the generated code file should be saved.
- `<target_language>`: The language in which the code should be generated ('c', 'cpp', 'js', 'python', 'md')

#### Examples:

- For C:
    ```
    python generator.py commands.json kinisi_commands.h --language c
    ```
- For C++:
    ```
    python generator.py commands.json kinisi_commands.h --language cpp
    ```
- For Markdown:
    ```
    python generator.py commands.json kinisi_commands.md --language md
    ```

### JSON File Format:

The provided JSON file should contain a list of command definitions and their details. Here's the structure of the JSON file:

1. **version**: Specifies the version of the command set.
2. **types**: An array of type definitions, which can be enums with their corresponding values.
   - **name**: The name of the type or enum.
   - **type**: Specifies whether it's an enum or another type.
   - **values**: An array of values for the enum type with a `name` and `value`.
3. **commands**: An array of command definitions.
   - **command**: The name of the command.
   - **code**: The byte code for the command.
   - **description**: A brief description of the command.
   - **properties**: An array of properties that the command accepts. Each property has:
     - **name**: The name of the property.
     - **type**: The type of the property, which should match one of the types defined in the `types` array.
     - **description**: A brief description of the property.
     - **range** (optional): A range of valid values for the property (if applicable).
   - **response** (optional): The expected response after executing the command. It has:
     - **name**: The name of the response property.
     - **type**: The type of the response property.
     - **description**: A brief description of the response.

**Note**: Always ensure that the input JSON is correctly formatted and adheres to the expected structure. The script performs validation, but it's always a good practice to double-check.
