"""算出「公共 C API 里还没被 Python 绑定」的函数清单（一次性工具）。

做 A13 时手工点数会错（文档里写的是 23，早期口头盘点写的是 24）。
这里直接扫头文件的 `JSDK_API` 声明 vs 包内 `lib.<name>` 的实际调用。
"""
import io
import re
import os
import glob

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HDRS = [os.path.join(ROOT, 'include/joint_sdk/joint_sdk.h'),
        os.path.join(ROOT, 'include/joint_sdk/jsdk_hal_builtin.h')]
PKG = os.path.join(ROOT, 'bindings/python/src/jsdk_can')

# 1) 头文件里所有 JSDK_API 声明的函数名
api = []
for h in HDRS:
    txt = io.open(h, encoding='utf-8').read()
    for m in re.finditer(r'JSDK_API\s+(?:[A-Za-z_][\w \t\*]*?)\s*(jsdk_\w+)\s*\(', txt):
        api.append(m.group(1))
api = sorted(set(api))

# 2) 包内实际用到的（lib.jsdk_xxx / lib['jsdk_xxx'] / ("jsdk_xxx", ...) 表驱动）
used = set()
for f in glob.glob(os.path.join(PKG, '**', '*.py'), recursive=True):
    txt = io.open(f, encoding='utf-8').read()
    used |= set(re.findall(r'\b(jsdk_\w+)', txt))

missing = [n for n in api if n not in used]
print('公共 API 总数   :', len(api))
print('包内出现过的    :', len(used & set(api)))
print('未绑定（缺口）  :', len(missing))
for n in missing:
    print('   -', n)
