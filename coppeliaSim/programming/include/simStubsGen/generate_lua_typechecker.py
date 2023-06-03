import argparse
import os
import sys
from parse import parse
from model import ParamTable

parser = argparse.ArgumentParser(description='Generate Lua typechecker.')
parser.add_argument('lua_file', type=str, default=None, help='the input lua file')
parser.add_argument('xml_file', type=str, default=None, help='the (lua) XML file')
parser.add_argument('out_file', type=str, default=None, help='the output lua file')
args = parser.parse_args()

if args is False:
    SystemExit

plugin = parse(args.xml_file)

with open(args.out_file, 'wt') as f:
    f.write(f"""-- sim{plugin.name} lua type-checking wrapper
-- (this file is automatically generated: do not edit)
require 'checkargs'

local sim{plugin.name}=require('{os.path.splitext(os.path.basename(args.lua_file))[0]}')

function sim{plugin.name}.__addTypeCheck()
    local function wrapFunc(funcName,wrapperGenerator)
        _G['sim{plugin.name}'][funcName]=wrapperGenerator(_G['sim{plugin.name}'][funcName])
    end
""")
    for cmd in plugin.commands:
        paramList = ','.join(p.name for p in cmd.all_params)
        f.write(f"""
    wrapFunc('{cmd.name}',function(origFunc)
        return function(...)
            {paramList and 'local ' or ''}{paramList}{paramList and '=' or ''}checkargsEx(
                {{level=1}},
                {{
""")
        for p in cmd.all_params:
            f.write(f"""                    {{type='{p.dtype}'""")
            if isinstance(p,ParamTable):
                if p.itype:
                    f.write(f""",item_type='{p.itype}'""")
                if p.size:
                    f.write(f""",size='{p.size}'""")
            if p.nullable:
                f.write(f""",nullable=true""")
            if p.default:
                f.write(f""",default={p.default}""")
            f.write(f"""}},
""")
        f.write(f"""                }},
                ...
            )
            local _rets={{origFunc({paramList})}}
            return table.unpack(_rets)
        end
    end)
""")
    f.write(f"""
    sim{plugin.name}.__addTypeCheck=nil
end

sim.registerScriptFuncHook('sysCall_init','sim{plugin.name}.__addTypeCheck',true)

return sim{plugin.name}
""")
