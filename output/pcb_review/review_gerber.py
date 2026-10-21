"""Read-only analysis of this export's linear C/R Gerber primitives.
Not a substitute for the CAD netlist/DRC. All coordinates are millimetres.
"""
import collections, json, math, re
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[2]
SRC=ROOT/'Gerber_PCB1_2026-10-21'
OUT=Path(__file__).resolve().parent
shapes=[]
def parse(path,layer):
    aps={}; ap=None; pos=(0.,0.); section=''
    for ln in path.read_text().splitlines():
        m=re.match(r'%ADD(\d+)([CR]),([\d.]+)(?:X([\d.]+))?\*%',ln)
        if m:
            aps[int(m[1])]=(m[2],float(m[3]),float(m[4] or m[3])); continue
        if ln.startswith('G04 '): section=ln[4:].strip('*'); continue
        m=re.fullmatch(r'G54D(\d+)\*',ln)
        if m: ap=aps[int(m[1])]; continue
        m=re.fullmatch(r'(?:G01)?X(-?\d+)Y(-?\d+)D0([123])\*',ln)
        if not m: continue
        p=(int(m[1])/1e5,int(m[2])/1e5); op=int(m[3])
        if op==1:
            assert ap[0]=='C',('unsupported stroke',ap)
            s={'layer':layer,'kind':'line','a':pos,'b':p,'r':ap[1]/2,'width':ap[1],'section':section}
        elif op==3:
            if ap[0]=='C': s={'layer':layer,'kind':'circle','a':p,'b':p,'r':ap[1]/2,'width':ap[1],'section':section}
            else:
                x,y=p; w,h=ap[1:]
                s={'layer':layer,'kind':'rect','poly':[(x-w/2,y-h/2),(x+w/2,y-h/2),(x+w/2,y+h/2),(x-w/2,y+h/2)],'width':w,'section':section}
        else: pos=p; continue
        pos=p
        if s['kind']=='rect':
            xs,ys=zip(*s['poly']); s['box']=(min(xs),min(ys),max(xs),max(ys))
        else:
            a,b,r=s['a'],s['b'],s['r']; s['box']=(min(a[0],b[0])-r,min(a[1],b[1])-r,max(a[0],b[0])+r,max(a[1],b[1])+r)
        shapes.append(s)
parse(SRC/'Gerber_TopLayer.GTL','T')
parse(SRC/'Gerber_BottomLayer.GBL','B')

def ptseg(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]; den=dx*dx+dy*dy
    t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den)) if den else 0
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
def cross(a,b,c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
def segdist(a,b,c,d):
    if max(min(a[0],b[0]),min(c[0],d[0]))<=min(max(a[0],b[0]),max(c[0],d[0]))+1e-12 and max(min(a[1],b[1]),min(c[1],d[1]))<=min(max(a[1],b[1]),max(c[1],d[1]))+1e-12:
        if cross(a,b,c)*cross(a,b,d)<=0 and cross(c,d,a)*cross(c,d,b)<=0: return 0.
    return min(ptseg(a,c,d),ptseg(b,c,d),ptseg(c,a,b),ptseg(d,a,b))
def contains(s,p):
    if s['kind']=='rect':
        x1,y1,x2,y2=s['box']; return x1-1e-5<=p[0]<=x2+1e-5 and y1-1e-5<=p[1]<=y2+1e-5
    return ptseg(p,s['a'],s['b'])<=s['r']+1e-5
def distance(s,t):
    if s['kind']!='rect' and t['kind']!='rect': return max(0,segdist(s['a'],s['b'],t['a'],t['b'])-s['r']-t['r'])
    if s['kind']!='rect': s,t=t,s
    pp=s['poly']; edges=list(zip(pp,pp[1:]+pp[:1]))
    if t['kind']=='rect':
        if contains(s,t['poly'][0]) or contains(t,pp[0]): return 0.
        qq=t['poly']; return min(segdist(a,b,c,d) for a,b in edges for c,d in zip(qq,qq[1:]+qq[:1]))
    if contains(s,t['a']) or contains(s,t['b']): return 0.
    return max(0,min(segdist(a,b,t['a'],t['b']) for a,b in edges)-t['r'])

parent=list(range(len(shapes)))
def find(x):
    while parent[x]!=x: parent[x]=parent[parent[x]]; x=parent[x]
    return x
def union(x,y):
    x,y=find(x),find(y)
    if x!=y: parent[y]=x
def boxgap(a,b): return max(a[0]-b[2],b[0]-a[2],a[1]-b[3],b[1]-a[3],0)
for i,s in enumerate(shapes):
    for k in range(i):
        t=shapes[k]
        if s['layer']==t['layer'] and boxgap(s['box'],t['box'])<=1e-5 and distance(s,t)<=1e-5: union(i,k)

holes=[]
for name in ['Drill_PTH_Through.DRL','Drill_PTH_Through_Via.DRL']:
    for ln in (SRC/name).read_text().splitlines():
        m=re.fullmatch(r'X(-?[\d.]+)Y(-?[\d.]+)',ln)
        if m: holes.append((float(m[1]),float(m[2])))
for p in holes:
    touching=[i for i,s in enumerate(shapes) if contains(s,p)]
    for i in touching[1:]: union(touching[0],i)

j=json.loads((SRC/'FlyingProbeTesting.json').read_text())
pins=[]; seen=set()
for rr in j['pins']['rows']:
    d=dict(zip(j['pins']['fields'],rr)); key=(d['PIN_NAME'],d['PIN_X'],d['PIN_Y'],d['NET_NAME'])
    if key in seen: continue
    seen.add(key); p=(d['PIN_X']*.0254,d['PIN_Y']*.0254)
    hits=[i for i,s in enumerate(shapes) if contains(s,p)]
    if not hits: raise ValueError(('unmatched pin',d['PIN_NAME'],p))
    roots=sorted(set(find(i) for i in hits))
    pins.append({'name':d['PIN_NAME'],'net':d['NET_NAME'],'xy':p,'roots':roots})
groups=collections.defaultdict(list)
for i in range(len(shapes)): groups[find(i)].append(i)
rootpins=collections.defaultdict(list)
for p in pins:
    for r in p['roots']: rootpins[r].append(p)
summaries=[]
for root,idx in groups.items():
    ps=rootpins[root]; nets=sorted(set(p['net'] for p in ps)); lines=[shapes[i] for i in idx if shapes[i]['kind']=='line']
    widths=collections.defaultdict(float)
    for s in lines: widths[str(s['width'])]+=math.dist(s['a'],s['b'])
    summaries.append({'root':root,'nets':nets,'pins':sorted(set(p['name'] for p in ps)),
        'tracks_mm_by_width':{w:round(l,3) for w,l in widths.items()},'shape_count':len(idx)})

# Nearest clearance between distinct connected copper components on the same layer.
clear=[]
for i,s in enumerate(shapes):
    for k in range(i):
        t=shapes[k]
        if find(i)==find(k) or s['layer']!=t['layer'] or boxgap(s['box'],t['box'])>0.3: continue
        dd=distance(s,t)
        if dd<0.3: clear.append({'gap_mm':round(dd,6),'layer':s['layer'],'shape_ids':[k,i],'roots':[find(k),find(i)]})
clear.sort(key=lambda d:d['gap_mm'])
report={'units':'mm','shape_count':len(shapes),'holes':len(holes),'groups':summaries,'clearances_below_0.3mm':clear,'pins':pins,'shapes':shapes}
(OUT/'geometry_analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')

# Render from above. Pin labels are attached only to connector first pads.
SCALE=19; xmin,ymin,xmax,ymax=2.145,25.843,132.145,125.843
W,H=round((xmax-xmin)*SCALE)+80,round((ymax-ymin)*SCALE)+80
def px(p): return (round(40+(p[0]-xmin)*SCALE),round(40+(ymax-p[1])*SCALE))
def draw_shape(d,s,color):
    if s['kind']=='rect': d.polygon([px(p) for p in s['poly']],fill=color)
    else:
        a,b,r=s['a'],s['b'],s['r']; w=max(1,round(2*r*SCALE)); d.line([px(a),px(b)],fill=color,width=w)
        for p in (a,b):
            x,y=px(p); rr=r*SCALE; d.ellipse((round(x-rr),round(y-rr),round(x+rr),round(y+rr)),fill=color)
def render(name,filter_fn,colors):
    im=Image.new('RGB',(W,H),(17,22,30)); d=ImageDraw.Draw(im)
    for i,s in enumerate(shapes):
        if filter_fn(i,s): draw_shape(d,s,colors(i,s))
    for p in holes:
        x,y=px(p); d.ellipse((x-2,y-2,x+2,y+2),fill='black')
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',16)
    for p in pins:
        if re.fullmatch(r'(H\d+|U\d+)_1',p['name']):
            x,y=px(p['xy']); d.text((x+4,y-21),p['name'].split('_')[0],fill='white',font=font)
    im.save(OUT/name)
render('copper_overlay.png',lambda i,s:True,lambda i,s:(235,65,55) if s['layer']=='T' else (50,95,230))
supply={'GND':(50,170,255),'12V':(255,190,35),'3.3V':(40,220,130),'+5V':(255,90,180),'8.4V':(200,150,255)}
rootsupply={g['root']:next((n for n in g['nets'] if n in supply),None) for g in summaries}
render('power_ground.png',lambda i,s:True,lambda i,s:supply.get(rootsupply[find(i)],(65,65,65)))
detail=Image.open(OUT/'power_ground.png').crop((60,1490,1110,1850))
canvas=Image.new('RGB',(1050,445),(17,22,30)); canvas.paste(detail,(0,85)); dd=ImageDraw.Draw(canvas)
ff=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',24)
dd.text((18,10),'Motor headers H1-H4: blue GND pads have no traces',font=ff,fill='white')
dd.text((18,43),'Orange = 12 V copper. Blue = isolated GND pads.',font=ff,fill=(80,185,255))
for p in pins:
    if p['name'] in ['H1_2','H2_2','H3_2','H4_2']:
        x,y=px(p['xy']); x-=60; y=y-1490+85
        dd.ellipse((x-23,y-23,x+23,y+23),outline='white',width=3)
        dd.text((x+26,y-36),p['name'].replace('_',' pin '),font=ff,fill='white')
canvas.save(OUT/'motor_ground_detail.png')
print(json.dumps({'shape_count':len(shapes),'hole_count':len(holes),'connected_groups':len(groups),
    'supplies':[g for g in summaries if any(n in supply for n in g['nets'])],
    'closest_clearances':clear[:12]},ensure_ascii=False,indent=2))
