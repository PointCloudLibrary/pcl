import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from PIL import Image


class TestImageCloud(unittest.TestCase):
    def test_point_extents_and_colors(self):
        cases = [
            ("L", [(1, 1, 0)], [(0.0, 0.0, 0)]),
            ("RGB", [], []),
            ("RGB", [(0, 0, (255, 0, 0)), (2, 0, (0, 0, 255))], [(-1, 0, 0xff0000), (1, 0, 0xff)]),
            ("RGB", [(1, 0, (0, 0, 0)), (1, 2, (0, 0, 0))], [(0, -1, 0), (0, 1, 0)]),
            ("RGB", [(0, 0, (1, 2, 3)), (2, 1, (7, 8, 9))], [(-1, -0.5, 0x010203), (1, 0.5, 0x070809)]),
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "image.png"
            for mode, pixels, expected in cases:
                with self.subTest(pixels=pixels):
                    image = Image.new(mode, (3, 3), "white")
                    for x, y, color in pixels:
                        image.putpixel((x, y), color)
                    image.save(path)
                    output = subprocess.check_output(
                        [sys.executable, str(Path(__file__).with_name("convert_image_to_point_cloud.py")), str(path)],
                        text=True,
                    )
                    header, data = output.split("DATA ascii\n")
                    self.assertIn("TYPE F F F U\n", header)
                    self.assertIn(f"POINTS {len(expected)}\n", header)
                    rows = [row.split() for row in data.splitlines()]
                    self.assertEqual([(float(x), float(y), int(rgb)) for x, y, _, rgb in rows], expected)


if __name__ == "__main__":
    unittest.main()
