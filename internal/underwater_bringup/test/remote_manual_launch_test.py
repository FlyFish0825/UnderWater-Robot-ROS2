"""Offline checks: the manual launch chain must preserve gateway safety options."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"


class RemoteManualLaunchTest(unittest.TestCase):
    def test_gateway_options_are_forwarded_at_each_include(self):
        for filename, included in (
            ("remote_manual.launch.xml", "manual.launch.xml"),
            ("manual.launch.xml", "base.launch.xml"),
        ):
            root = ET.parse(LAUNCH_DIR / filename).getroot()
            declarations = {arg.attrib["name"]: arg for arg in root.findall("arg")}
            includes = [node for node in root.findall("include")
                        if node.attrib["file"].endswith("/" + included)]
            self.assertEqual(len(includes), 1)
            forwarded = {arg.attrib["name"]: arg.attrib.get("value")
                         for arg in includes[0].findall("arg")}
            for name, default in (("motor_tx", "false"),
                                  ("tcp_allow_tx", "false"),
                                  ("tcp_bind_address", "0.0.0.0")):
                with self.subTest(launch=filename, option=name):
                    self.assertIn(name, declarations)
                    self.assertEqual(declarations[name].attrib.get("default"), default)
                    self.assertEqual(forwarded.get(name), "$(var " + name + ")")

    def test_remote_endpoint_and_namespace_are_explicit(self):
        root = ET.parse(LAUNCH_DIR / "remote_manual.launch.xml").getroot()
        declarations = {arg.attrib["name"]: arg.attrib.get("default")
                        for arg in root.findall("arg")}
        self.assertEqual(declarations["namespace"], "underwater_robot")
        self.assertEqual(declarations["remote_manual_port"], "9003")
        nodes = [node for node in root.findall("node")
                 if node.attrib.get("exec") == "remote_manual_node"]
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].attrib["namespace"], "$(var namespace)")
        params = {param.attrib["name"]: param.attrib.get("value")
                  for param in nodes[0].findall("param")}
        self.assertEqual(params["bind_address"], "$(var remote_bind_address)")
        self.assertEqual(params["port"], "$(var remote_manual_port)")


if __name__ == "__main__":
    unittest.main()
