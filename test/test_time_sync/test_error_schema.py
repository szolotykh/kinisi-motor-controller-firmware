# ------------------------------------------------------------
# File name: test_error_schema.py
# ------------------------------------------------------------
"""Reject ambiguous error catalogs and undocumented command error references."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
root = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('generator', root / 'tools/commands_generator/generator.py')
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class ErrorSchemaTests(unittest.TestCase):
    def setUp(self):
        """Load an independent copy of commands.json for each schema validation test."""
        self.schema = json.loads((root / 'commands.json').read_text())

    def test_invalid_codes_and_names(self):
        """Reject reserved names, invalid wire codes, and duplicate error definitions."""
        self.assertEqual(generator.validate_json(self.schema), [])
        for field, value in [('code', 0), ('code', 256), ('code', True), ('name', 'OK'), ('name', 'bad-name')]:
            schema = copy.deepcopy(self.schema)
            schema['error_codes'][0][field] = value
            self.assertTrue(generator.validate_json(schema), (field, value))
        self.schema['error_codes'].append(copy.deepcopy(self.schema['error_codes'][0]))
        self.assertTrue(generator.validate_json(self.schema))

    def test_missing_unknown_and_duplicate_references(self):
        """Reject malformed per-command error arrays and undefined or repeated names."""
        for value in [None, 'INVALID_LENGTH', ['UNDEFINED_ERROR'], ['INVALID_LENGTH', 'INVALID_LENGTH']]:
            schema = copy.deepcopy(self.schema)
            schema['commands'][0]['errors'] = value
            self.assertTrue(generator.validate_json(schema), value)

    def test_cli_rejects_without_overwriting_output(self):
        """Verify schema validation failure preserves the previous generated output file."""
        del self.schema['commands'][0]['errors']
        with tempfile.TemporaryDirectory() as folder:
            source, output = Path(folder) / 'bad.json', Path(folder) / 'commands.h'
            source.write_text(json.dumps(self.schema))
            output.write_text('preserve previous generated file')
            result = subprocess.run([sys.executable, str(root / 'tools/commands_generator/generator.py'),
                                     str(source), str(output), '--language', 'c'], capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(output.read_text(), 'preserve previous generated file')


if __name__ == '__main__':
    unittest.main()
