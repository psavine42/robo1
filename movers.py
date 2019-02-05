from robolink import *
from robodk import *
import numpy as np
import networkx as nx
import trimesh


def cleanup(parentnodes, name):
    """Delete all child items whose name starts with \"ball\",
    from the provided list of parent items."""
    todelete = []
    for item in parentnodes:
        todelete = todelete + item.Childs()

    for item in todelete:
        if item.Name().startswith(name):
            item.Delete()


# MAYAN------------------------------------------------------
def generate_box(w=100, l=1000, h=100):
    """
    2x4s are generated and place in load queue

    create frame 4-> basic frame
    A-Frame
    Balloon wall
        

    Method 1:
        - Generate stls to be loaded latter.

    Returns:
        - list of objects with predicted positions
        -
    """
    import shapely.geometry

    ls = shapely.geometry.Polygon(
        [(-l/2, -w/2), (-l/2, w/2), (l/2, w/2), (l/2, -w/2)]
    )
    tri = trimesh.primitives.Extrusion(polygon=ls, height=h)
    tris = np.array(tri.triangles.tolist())
    # return tri.triangles.reshape((-1, 3)).tolist()
    tris[:, :, -1] -= h/2
    return tris.reshape((-1, 3))


def bbox_at(rdk, w, l, h, xyz_xyz=None, name='Shape') -> Item:
    """
    place a box at a place, then move it to
    """
    if xyz_xyz is None:
        xyz_xyz = [0, 0, 0, 0, 0, 0]
    tris = generate_box(w, l, h)
    item = rdk.AddShape(tris.tolist())
    pose = Pose(*xyz_xyz)
    item.setPose(pose)
    item.setName(name)
    return item


def make_table(rdk, lw, name='table1'):
    root = rdk.ItemList(filter=3)[0]
    rdk.AddFrame(name)


# def frame
"""
reading an input object, and generating the graph.
    - parametric objects:
        - boxlike : base

*** should the 

layout of members on assembly line
    - largest surface.
    - longest axis
    - translation:end-pose -> assembly-pose
    - 
assembly of pieces (robot program)
    - read 
"""


class Extrusion(object):
    def __init__(self, x, y, z):
        main_axis = max([x, y, z])
        inds = np.argsort([x, y, z])
        self._main = inds[0]
        self._secd = inds[1]
        self.x, self.y, self.z = x, y, z
        self.geom = None

    def align_to_assembly(self):
        return

    def geom(self):
        if self.geom is None:
            self.geom = generate_box(self.x, self.y, self.z)



def translate_to_al(source, target):
    # origin = Pose(bx * (i * offs), h/2, by/2, 90, 0, 90)
    hpi = pi / 180
    x, y, z, rx, ry, rz = Pose_2_TxyzRxyz(target)
    new_target = source * transl(x, y, z) * rotx(rx * hpi) * roty(ry * hpi) * rotz(rz * hpi)
    return new_target


def make_a_config(rdk: Robolink):
    bx, by, h = 50, 30, 1000
    rdk.Render(False)
    offs = 1 + (bx + 5) / bx
    table = rdk.Item('Table 1')
    table2 = rdk.Item('Table 2')
    cleanup([table, table2], 'obj')

    # target locations - where these should endup
    sx, sy = 300, h/2
    inits = [[sx, sy,     by,         -180, 0, -180],  # horizantal
             [sx, by/2,   h/2 + by,   -90,  0, 0],     # vertical
             [sx, h-by/2, h/2 + by,   -90,  0, 0],
             [sx, sy,     h + 3*by/2, -180, 0, -180],  # horizantal-top
             ]

    res = []
    for i, li in enumerate(inits):
        itm = bbox_at(rdk, bx, by, h, name='obj {}'.format(i))

        # set parent move it to its origin
        itm.setParent(table)
        origin = Pose(bx * (i * offs), h/2, by/2, 90, 0, 90)

        itm.setPose(origin)
        tgt = TargetItem(itm, li, geom=[bx, by, h])
        res.append(tgt)

    rdk.Render(True)
    import networkx as nx
    g = nx.DiGraph()
    g.add_edge('obj 0', 'obj 1', join='on')
    g.add_edge('obj 0', 'obj 2', join='on')
    g.add_edge('obj 1', 'obj 3', join='on')
    g.add_edge('obj 2', 'obj 3', join='on')
    root = 'obj 0'
    problem = Problem(res, g, root)
    problem.loading = table
    return problem


def compute_order(g):
    nx.


class TargetItem(object):
    def __init__(self, item, target, geom=None):
        self._item = item
        self._target = target
        self._geom = geom
        self._done = False

    def pickup_pose(self):
        pose = self._item.Pose()
        x, y, z = Pose_2_TxyzRxyz(pose)[0:3]
        return Pose(x, y, z, -180, 0, -180)

    def target_pose(self):
        # pose = self._item.Pose()

        return Pose(*self._target)

    @property
    def item(self):
        return self._item

    @property
    def target(self):
        return self._target

    def set_done(self):
        self._done = not self._done

    def done(self):
        return self._done


class Problem(object):
    def __init__(self, items, graph, root):
        self._items = items
        self._graph = graph
        self.loading = None

    @property
    def items(self):
        return self._items


def pickup(robot, t1, obj, tool=None):
    base = Pose(0, 0, 50, -180, 0, -180)
    if tool is None:
        tool = robot.Childs()[0]
    robot.setPoseFrame(t1)

    pick = obj.pickup_pose()
    robot.MoveJ(base)  #
    robot.MoveJ(pick)
    tool.AttachClosest()
    return True


def pick_and_place(robot: Item, t1: Item, t2: Item, obj: TargetItem):
    base = Pose(0, 0, 50, -180, 0, -180)
    tool = robot.Childs()[0]
    succ = pickup(robot, t1, obj, tool)

    if succ is False:
        return False
    # move to target
    finl = obj.target_pose()
    robot.setPoseFrame(t2)
    robot.MoveJ(base)
    robot.MoveJ(finl)
    tool.DetachAll(parent=t2)
    return True


class Assemble(object):
    def __init__(self, problem, rdk: Robolink):
        self._problem = problem
        self.rdk = rdk

    def reset(self):
        self.rdk = Robolink()

    def solve(self):
        if self._problem is None:
            print('no problem')
            return

        robot = self.rdk.ItemList(filter=2)[0]
        tool = robot.Childs()[0]
        sett = self.rdk.Item('Table 2')
        home = robot.Pose()

        robot.MoveJ([0, 0, 0, 00, -90, -90])
        load = self._problem.loading
        q = self._problem.items

        while q:
            el = q.pop(0)
            # pickup thing
            res = pick_and_place(robot, load, sett, el)
            el.set_done()

        robot.MoveL(home)
        # procedure to put on final location
        return


def check_pos(rdk):
    """
    tgts = generate targets (2x4s)

    init pickup [p1]

    while not done:
        A
        1) create pose on object for pickup -> p1
        2) grab
        3) move to target

    """
    rdk.Render(False)
    problem = make_a_config(rdk)
    rdk.Render(True)
    assemble = Assemble(problem, rdk)

    # assemble.solve()
    return problem, assemble



