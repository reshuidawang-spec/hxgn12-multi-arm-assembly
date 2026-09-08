import unittest
from unittest.mock import Mock
import numpy as np
from scripts.build_cabinet_product_scene import _world_bbox, build_paired_targets, _load_manifest
from scripts.preprocess_cabinet_models import transform
from scripts.run_8arm_cabinet_assembly import part_gripper_gap
from sim_bridge.cabinet_geometry import axis_intersections, triangles, flat_patch_height, require_open_top_shell, vacuum_grasp_point


class CabinetGeometryFrameTests(unittest.TestCase):
    def test_roof_is_rejected_even_with_correct_bounding_dimensions(self):
        mesh=np.array([[[-1,-1,1],[1,-1,1],[0,1,1]],
                       [[-1,-1,0],[0,1,0],[1,-1,0]]],dtype=float)
        from unittest.mock import patch
        with patch('sim_bridge.cabinet_geometry.triangles',return_value=mesh):
            with self.assertRaisesRegex(RuntimeError,'solid shell roof'):
                require_open_top_shell()

    def test_world_bounds_follow_mesh_not_rotated_shape_bb(self):
        sim = Mock()
        sim.getShapeMesh.return_value = ([-.2262,-.1202,0,.2263,.1203,.142], [], [])
        sim.getObjectMatrix.return_value = [0,0,1,0,0,1,0,0,-1,0,0,0]
        lo, hi = _world_bbox(sim, [1], [], [])
        np.testing.assert_allclose(hi-lo, [.142,.2405,.4525])
        sim.getObjectFloatParam.assert_not_called()

    def test_front_of_original_cad_faces_up(self):
        points = np.array([[457,243.6,3.2,457,243.6,287.2,457,243.6,3.2]])
        actual = transform(points).reshape(-1,3)
        self.assertAlmostEqual(actual[0,2], .142)
        self.assertAlmostEqual(actual[1,2], 0)
        self.assertGreater(actual[0,2], actual[1,2])
        require_open_top_shell()

    def test_no_door_and_filter_installed_inside_open_shell(self):
        m = _load_manifest()
        self.assertEqual(m['frame'], 'cabinet_open_up_v2')
        t = build_paired_targets(m)
        self.assertFalse(any('DOOR' in k or 'LATCH' in k for k in t))
        self.assertLess(t['R8_FILTER_PLACE'][0][2], .412)
        self.assertIn('mounting_panel',m['parts'])
        self.assertNotIn('door',m['parts'])

    def test_gripper_width_tracks_each_actual_part(self):
        m = _load_manifest()['parts']
        for robot, ids in [('R4', ['psu','servo','eds']),('R6',['contactor','breaker','com5'])]:
            for part in ids:
                info=m[part]
                yz=tuple((info['bbox_hi'][i]+info['bbox_lo'][i])/2 for i in (1,2))
                hits=axis_intersections(triangles(part),yz,0)
                self.assertGreaterEqual(len(hits),2)
                self.assertAlmostEqual(part_gripper_gap(robot,part),hits[-1]-hits[0]-.0005)

    def test_suction_patches_have_real_flat_material(self):
        for part in ('plc','dma','filter'):
            point=vacuum_grasp_point(part)
            self.assertAlmostEqual(flat_patch_height(part,point[:2],.006),point[2])

if __name__ == '__main__':
    unittest.main()
