import json
from pathlib import Path
import subprocess
import sys
import unittest

from app import binary_protocol_generated as generated


ROOT = Path(__file__).resolve().parents[2]


class ProtocolSchemaTests(unittest.TestCase):
    def test_generated_files_are_current(self):
        subprocess.run(
            [sys.executable, str(ROOT / "tools" / "generate_rpi_protocol.py"), "--check"],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        )

    def test_schema_matches_runtime_constants(self):
        schema = json.loads(
            (ROOT / "protocol" / "rpi_binary_protocol.json").read_text(encoding="utf-8")
        )
        self.assertEqual(generated.HOST_PROTOCOL_VERSION, schema["host_protocol_version"])
        self.assertEqual(generated.VERSION, schema["wire_version"])
        self.assertEqual(generated.MAX_PAYLOAD, schema["max_payload"])
        for name, value in schema["commands"].items():
            self.assertEqual(int(generated.Command[name]), value)
        for name, value in schema["message_types"].items():
            self.assertEqual(int(generated.MessageType[name]), value)


if __name__ == "__main__":
    unittest.main()
