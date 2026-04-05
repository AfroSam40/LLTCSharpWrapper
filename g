from pythonnet import load
load("coreclr")

import clr
import sys

sys.path.append(r"C:\path\to\your\dll\folder")
clr.AddReference("3DLibsDll")

# example only
# from YourNamespace import YourClass
# obj = YourClass()
# print(obj.SomeMethod())