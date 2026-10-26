# ------------------------------------------------------------
# File name: test_error_schema.py
# ------------------------------------------------------------
"""Reject ambiguous error catalogs and undocumented command error references."""
import copy
import importlib.util
import json
import re
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

    def test_topic_registry_and_wire_selector_validation(self):
        for field, value in [('id', 0), ('id', 256), ('id', True), ('name', 'invalid-name'),
                             ('payload_type', 'undefined'), ('payload_type', 'imu_sample'), ('index_max', 256),
                             ('min_interval_ms', 0), ('min_interval_ms', 3600001)]:
            schema = copy.deepcopy(self.schema)
            schema['topics'][0][field] = value
            self.assertTrue(generator.validate_json(schema), (field, value))
        schema = copy.deepcopy(self.schema)
        schema['topics'].append(copy.deepcopy(schema['topics'][0]))
        self.assertTrue(generator.validate_json(schema))
        schema = copy.deepcopy(self.schema)
        event = next(c for c in schema['commands'] if c['command'] == 'TELEMETRY_EVENT')
        event['topic_payload']['discriminator'] = 'unknown'
        self.assertTrue(generator.validate_json(schema))

    def test_generated_topic_sizes_and_documentation(self):
        code = generator.generate_c_header_file(self.schema)
        self.assertIn('TOPIC_ENCODER_ODOMETRY = 1', code)
        self.assertIn('TOPIC_IMU = 4', code)
        self.assertIn('sizeof(imu_telemetry_sample)', code)
        self.assertIn('telemetry_payload_size(command->properties.subscribe.topic_id) != 0', code)
        docs = generator.generate_md_file(self.schema)
        self.assertIn('total payload length varies by topic', docs)
        self.assertIn('2 bytes fixed prefix + topic payload', docs)
        self.assertIn('| 4 | ' + chr(96) + 'IMU' + chr(96), docs)

    def test_retirement_metadata_validation(self):
        original = next(c for c in self.schema['commands'] if c['command'] == 'SUBSCRIBE_ODOMETRY')['retired']
        invalid = [None, True, {}, {**original, 'unknown': 1}]
        for field, value in [('since', '2.6'), ('since', '2.8.0'), ('since', True),
                             ('since', '02.6.0'), ('removal_major', 2), ('removal_major', True),
                             ('removal_major', 256), ('replacement', 'MISSING'),
                             ('replacement', 'SUBSCRIBE_ODOMETRY'), ('replacement', 'UNSUBSCRIBE_ODOMETRY'),
                             ('replacement', 'TELEMETRY_EVENT'), ('replacement', [])]:
            invalid.append({**original, field: value})
        for retired in invalid:
            schema = copy.deepcopy(self.schema)
            next(c for c in schema['commands'] if c['command'] == 'SUBSCRIBE_ODOMETRY')['retired'] = retired
            with self.subTest(retired=retired):
                self.assertTrue(generator.validate_json(schema))

    def test_retirement_annotations_preserve_generated_wire_code(self):
        active = copy.deepcopy(self.schema)
        for command in active['commands']:
            command.pop('retired', None)
        strip_comments = lambda code: re.sub(r'\s+', '', re.sub(r'/\*.*?\*/|//[^\n]*', '', code, flags=re.S))
        for generate in [generator.generate_c_header_file, generator.generate_cpp_code]:
            annotated = generate(self.schema)
            self.assertIn('Retired since protocol 2.6.0. Use SUBSCRIBE.', annotated)
            self.assertEqual(strip_comments(annotated), strip_comments(generate(active)))
        docs = generator.generate_md_file(self.schema)
        self.assertIn('## Command lifecycle', docs)
        self.assertIn('**Retired since protocol 2.6.0.**', docs)
        self.assertIn('eligible for removal in protocol 3.0.0.', docs)
        retired = {c['command'] for c in self.schema['commands'] if 'retired' in c}
        self.assertEqual(retired, {'SUBSCRIBE_ODOMETRY', 'UNSUBSCRIBE_ODOMETRY',
                                   'ENCODER_ODOMETRY_EVENT', 'PLATFORM_ODOMETRY_EVENT'})

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
