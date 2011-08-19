import sys
import os

from Cheetah.Template import Template

srcdir=sys.argv[1]
s=str(Template(open(os.path.join(srcdir, "cloud.tmpl")).read(), searchList=[{'type': 'PointXYZ', 'members' : ['x', 'y', 'z']}]))
open(os.path.join(sys.argv[2], "pygen.i"), "w").write(s)
