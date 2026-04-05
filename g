from pythonnet import load
load("coreclr")

import clr
from System import String

s = String("hello from .NET")
print(s)