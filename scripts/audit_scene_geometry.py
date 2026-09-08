"""Read-only scene inventory; reports actual mesh bounds, not oriented BB extrema."""
import argparse
import json
from pathlib import Path
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

INVENTORY = r'''
local sim=require('sim')
local out={objects={},triangles=0}
for _,h in ipairs(sim.getObjectsInTree(sim.handle_scene,sim.handle_all,0)) do
    local kind=sim.getObjectType(h)
    local row={handle=h,alias=sim.getObjectAlias(h,0),parent=sim.getObjectParent(h),kind=kind}
    if kind==sim.object_shape_type then
        local vertices,indices=sim.getShapeMesh(h)
        row.triangles=#indices/3
        out.triangles=out.triangles+row.triangles
        row.visible=sim.getObjectInt32Param(h,sim.objintparam_visibility_layer)
        row.respondable=sim.getObjectInt32Param(h,sim.shapeintparam_respondable)
        local m=sim.getObjectMatrix(h,sim.handle_world)
        local lo={math.huge,math.huge,math.huge}
        local hi={-math.huge,-math.huge,-math.huge}
        for i=1,#vertices,3 do
            local p=sim.multiplyVector(m,{vertices[i],vertices[i+1],vertices[i+2]})
            for j=1,3 do lo[j]=math.min(lo[j],p[j]);hi[j]=math.max(hi[j],p[j]) end
        end
        row.lo=lo;row.hi=hi
    end
    out.objects[#out.objects+1]=row
end
return out
'''

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',type=int,default=23000)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    c=RemoteAPIClient('127.0.0.1',a.port)
    s=c.require('sim')
    helper=s.createScript(s.scripttype_customization,'function inventory()\n'+INVENTORY+'\nend',0)
    try:
        s.initScript(helper)
        report=s.callScriptFunction('inventory',helper)
        report['objects']=[o for o in report['objects'] if o['handle']!=helper]
    finally:
        s.removeObjects([helper])
    report['scene']=s.getStringParam(s.stringparam_scene_path_and_name)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    shapes=sorted((o for o in report['objects'] if 'triangles' in o),key=lambda o:o['triangles'],reverse=True)
    print(json.dumps({'objects':len(report['objects']),'triangles':report['triangles'],'largest_meshes':shapes[:12]},indent=2))

if __name__=='__main__':
    main()
