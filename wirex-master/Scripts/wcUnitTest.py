# -*- coding: utf-8 -*-
# 
# WireX
#
# Copyright (c) 2006-2019 Andreas Pott
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# 
"""
Created on 22.09.2017

@author: Andreas Pott

This is the baseline to develop a unitest framework for WireLib. As
scripting is a convinient way to write unit tests, the following collection is
a draft for setting up some test for WireLib through the WiPy interface.

Tests are structured as follows. Methods with tests must start with the prefix
"test_". This is followed by the WiPy modules that is tested, i.e. where the
tested functions are stored in WiPy. The remained of the function name characterizes
what is tested.

Some tests are called compliance* tests where * is a number such as 644. These
tests check if numerical values determined with a older version of WiPy are reproduced.
Basically, we do not know if these values are correct. However, failure if such test
indicate that the numerical behaviour of the implementation is changed.
"""

import unittest
import os
import sys
import WiPy
# the following imports are redundant but make some IDE's happy
from WiPy import Ikin, Irobot, Iws, Icontrol

VERSION_INFO = """
|---------------------------------------------------------------------
| This is the WireX Unit Testing Framework.
| Testing WiPy Version: """ + WiPy.version() + """
| (c)opyright 2018-2019, Andreas Pott
|---------------------------------------------------------------------
"""

VERSION_REMARK = """
|---------------------------------------------------------------------
| Warning: The current version of the unit tests detects an error in 
| test_Ikin_forwardkinematicsPulley
| (consistency of forward/inverse pulley kinematics). This bug is 
| known but not fixed. 
|---------------------------------------------------------------------
"""

class WiPyMethods(unittest.TestCase):
    """
    test the WiPy module; functions containing test must start with "test_" to be autoexecuted by the 
    framework
    """
    def setup(self):
        """ Set some class wide testing constants such as pre-define benchmark
        poses
        """
        self.POS1 = [0.1, 0.2, 1]
        self.ORI1_Angles = [0, 0, 0]
        self.ORI1_R = [[1, 0, 0], [0, 0, 1], [0, 0, 1]]

    def test_WiPy_Robot_Creation(self):
        self.assertEqual(WiPy.createRobot(), 55, msg="robot creation not successful")

    def test_Irobot_basicGetter(self):
        self.assertEqual(Irobot.applyModel('IPAnema1Design'), True, msg="apply geometry model failed")
        self.assertEqual(Irobot.getNow(), 8, msg="for build-in model number of wires mismatch")
        self.assertEqual(Irobot.getDof(), 6, msg="for build-in model degrees-of-freedom mismatch")

    def test_Irobot_loadsave(self):
        self.assertAlmostEqual(Irobot.save('__robot.wcrfx'), True)
        self.assertAlmostEqual(Irobot.load('__robot.wcrfx'), True)
        os.remove('__robot.wcrfx')

    def test_Irobot_applyStandardModel(self):
        self.assertEqual(Irobot.applyModel('IPAnema1Design'), True)

    def test_Irobot_applyAllModels(self):
#        """Test all 8 cable models"""
        models = ["ParametricBoxFrame",
            "ParametricBoxPlatform",
            "IPAnema1Design",
            "IPAnema2Frame",
            "IPAnema3Design",
            "CoGiRo8Design",
            "MotionSimulatorDesign",
            "FrameTransformator",
            "PlatformTransformator",
            "Kawamura8Design",
            "Segesta8Design",
            "PointPlatform",
            "PointFrame",
            "IPAnema1Design"]
        for model in models:
            self.assertEqual(Irobot.applyModel(model), True, msg="Error while applying model: "+model)

    def test_Ikin_inversekinematics(self):
        self.assertTrue(Irobot.applyModel('IPAnema1Design'))
        self.assertEqual(Ikin.setPosition(0, 0, 1), None)
        self.assertEqual(Ikin.setRotationXYZ(0, 0, 0), None)
        # precomputed results fro IPAnema 1 robot
        self.assertEqual(Ikin.inverseKinematics2(), [2.6148040079516472, 2.6148040079516472,
                         2.6148040079516472, 2.6148040079516472, 2.6148040079516472,
                         2.6148040079516472, 2.6148040079516472, 2.6148040079516472])

    def test_Ikin_forwardkinematics(self):
        self.assertTrue(Irobot.applyModel('IPAnema1Design'))
        self.assertEqual(Ikin.setPosition(0, 0, 1), None)
        self.assertEqual(Ikin.setRotationXYZ(0, 0, 0), None)
        l = Ikin.inverseKinematics2()
        Ikin.setSolverAlgorithm(1) # 1 = LM solver without Jacobian
        self.assertEqual(Ikin.forwardKinematics(l)[0:3], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.assertAlmostEqual(Ikin.forwardKinematics(l)[3], [0, 0, 1])

    def test_Ikin_forwardkinematicsEigen(self):
        import numpy
        eps = 1e-6  # maximum acceptable vector norm of the errors in position and vectors in orientation matrix
        self.assertTrue(Irobot.applyModel('IPAnema1Design'))
        self.assertEqual(Ikin.setPosition(0.1, 0.2, 1), None)
        self.assertEqual(Ikin.setRotationXYZ(0, 0, 0), None)
        l = Ikin.inverseKinematics2()
        # 5 = CMINPACK from eigen3 with LM solver with Jacobian
        Ikin.setSolverAlgorithm(5) 
        R1,R2,R3,r = numpy.array( Ikin.forwardKinematics(l) )
        self.assertLess(numpy.linalg.norm(R1 - [1, 0, 0]),eps, msg="Error in first col of R")
        self.assertLess(numpy.linalg.norm(R2 - [0, 1, 0]),eps, msg="Error in second col of R")
        self.assertLess(numpy.linalg.norm(R3 - [0, 0, 1]),eps, msg="Error in third col of R")
        self.assertLess(numpy.linalg.norm(r  - [0.1, 0.2, 1]),eps, msg="Error in position r")
        """
        self.assertAlmostEqual(Ikin.forwardKinematics(l)[3], [0, 0, 1])
        self.assertAlmostEqual(Ikin.forwardKinematics(l)[0], [1, 0, 0])
        self.assertAlmostEqual(Ikin.forwardKinematics(l)[1], [0, 1, 0])
        self.assertAlmostEqual(Ikin.forwardKinematics(l)[2], [0, 0, 1])
        self.assertAlmostEqual(Ikin.forwardKinematics(l)[3], [0.1, 0.2, 1])
        """

    def test_Ikin_forwardkinematicsPulley(self):
        """
        This tests has some issues and is deactived as l=Ikin.inverseKinematicsPulley()
        returns currently some NAN entries
        """
        self.assertTrue(Irobot.applyModel('IPAnema1Design'))
        self.assertEqual(Ikin.setPosition(0, 0, 1), None)
        self.assertEqual(Ikin.setRotationXYZ(0, 0, 0), None)
        Ikin.setSolverAlgorithm(0) # LM solver without Jacobian
        l = Ikin.inverseKinematicsPulley()
        self.assertEqual(Ikin.forwardKinematicsPulley(l)[0:3], [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                         msg="pulley kinematics forward-inverse mismatch in orientation")
        self.assertAlmostEqual(Ikin.forwardKinematicsPulley(l)[3], [0, 0, 1],
                         msg="pulley kinematics forward-inverse mismatch in position")

    def test_Ikin_forwardkinematicsPulley_AlmostEqual(self):
        import numpy
        self.assertTrue(Irobot.applyModel('IPAnema1Design'))
        self.assertEqual(Ikin.setPosition(0, 0, 1), None)
        self.assertEqual(Ikin.setRotationXYZ(0, 0, 0), None)
        Ikin.setSolverAlgorithm(0) # LM solver without Jacobian
        l = Ikin.inverseKinematicsPulley()
        R1,R2,R3,r = numpy.array( Ikin.forwardKinematicsPulley(l) )
        self.assertLess(numpy.linalg.norm(R1 - [1, 0, 0]),1e-3, msg="Error in first col of R")
        self.assertLess(numpy.linalg.norm(R2 - [0, 1, 0]),1e-3, msg="Error in second col of R")
        self.assertLess(numpy.linalg.norm(R3 - [0, 0, 1]),1e-3, msg="Error in third col of R")
        self.assertLess(numpy.linalg.norm(r  - [0, 0, 1]),1e-2, msg="Error in position r")

    def test_Ikin_getsetPosition(self):
        pos = [0.1, 0.2, 1.1]
        self.assertEqual(Ikin.setPosition(*pos), None)
        self.assertEqual(Ikin.getPosition(), pos)
        
    def test_Ikin_getForceDistribution_consistance(self):
        import numpy
        self.assertTrue(Irobot.applyModel('IPAnema1Design'))
        w = numpy.array([1, 1, 1, 0, 0, 0])
        self.assertIsNone(Iws.setWrench(*w))
        f = numpy.array(Irobot.getForceDistribution(0.1, 0.2, 1))
        AT = numpy.array(Ikin.getStructureMatrixMatrix())
        # test if structure matrix is fulfilled (i.e. the norm of the evaluation is sufficiently small)
        self.assertLess(numpy.linalg.norm(AT.transpose().dot(f)+w), 1e-10)
        
    def test_Irobot_getForceDistribution_referencevalue(self):
        self.assertTrue(Irobot.applyModel('IPAnema1Design'))
        self.assertIsNone(Iws.setWrench(0, 0, 0, 0, 0, 0))
        self.assertEqual(Irobot.getForceDistribution(0, 0, 1), [5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5])
        
    def test_Ikin_getPotentialEnergy_consistence(self):
        self.assertTrue(Irobot.applyModel('IPAnema1Design'))
        self.assertEqual(Ikin.setPosition(0, 0, 1), None)
        self.assertEqual(Ikin.setRotationXYZ(0, 0, 0), None)
        l1 = 8*[2.6148040079516472]       # nominal values
        l2 = 8*[2.6148040079516472+0.001] # longer cable --> less potential energy
        l3 = 8*[2.6148040079516472-0.001] # shorter cable --> more potential energy
        self.assertGreaterEqual(Ikin.getPotentialEnergy(l1),Ikin.getPotentialEnergy(l2))
        self.assertLess(Ikin.getPotentialEnergy(l1),Ikin.getPotentialEnergy(l3))

    def test_Ikin_getPotentialEnergy_consistence_rotation(self):
        self.assertTrue(Irobot.applyModel('IPAnema1Design'))
        self.assertEqual(Ikin.setPosition(0, 0, 1), None)
        l1 = 8*[2.6148040079516472]       # nominal values
        self.assertEqual(Ikin.setRotationXYZ(0, 0, 0), None)
        H1 = Ikin.getPotentialEnergy(l1)
        self.assertEqual(Ikin.setRotationXYZ(0.1, 0, 0), None)
        H2 = Ikin.getPotentialEnergy(l1)
        # For two orientations energy must be different
        self.assertNotEqual(H1,H2)

    def test_Ikin_getPotentialEnergy_wrongParameter(self):
        self.assertTrue(Irobot.applyModel('IPAnema1Design'))
        l1 = 7*[2.6148040079516472]       # nominal values
        self.assertEqual(Ikin.getPotentialEnergy(l1), False)

    def test_Irobot_GetterSetter(self):
        ai = [-2, -1.5, 2]
        Irobot.setBase(0, *ai)
        self.assertEquals(Irobot.getBase(0), ai)

    def test_Iws_GetterSetter(self):
        eps = 0.01
        self.assertIsNone(Iws.setEps(eps))
        self.assertEqual(Iws.getEps(), eps)
        self.assertNotEqual(Iws.getEps(), eps*2)
        limits = [2, 20]
        Iws.setForceLimits(*limits)
        self.assertEqual(Iws.getForceLimits(), limits)
        center = [0.1, 0.2, 1.0]
        Iws.setProjectionCenter(*center)
        self.assertEqual(Iws.getProjectionCenter(), center)

    def test_Ikin_structureMatrix_compliance644(self):
        """
        Test for reproduction of results computed with an older version of WiPy
        here: structure matrix, reference data determined on 06.12.2017
        """
        AT= [[-0.7881678136715315, 0.4790823965454407, 0.3863567714076135, 0.023181406284456807, 0.023181406284456807, 0.018545125027565453],
            [0.7560310364971451, 0.5094991767698152, 0.4108864328788832, 0.024653185972732992, -0.024653185972732992, -0.014791911583639798],
            [0.6917488766649749, -0.6165587813753037, 0.37595047644835594, -0.022557028586901357, -0.022557028586901357, 0.004511405717380269],
            [-0.728051577325381, -0.5852963660851102, 0.35688802810067694, -0.021413281686040616, 0.021413281686040616, -0.00856531267441625],
            [-0.7881678136715315, 0.4790823965454407, -0.3863567714076135, -0.023181406284456807, -0.023181406284456807, 0.018545125027565453],
            [0.7560310364971451, 0.5094991767698152, -0.4108864328788832, -0.024653185972732992, 0.024653185972732992, -0.014791911583639798],
            [0.6917488766649749, -0.6165587813753037, -0.37595047644835594, 0.022557028586901357, 0.022557028586901357, 0.004511405717380269],
            [-0.728051577325381, -0.5852963660851102, -0.35688802810067694, 0.021413281686040616, -0.021413281686040616, -0.00856531267441625]]
        Irobot.applyModel("IPAnema1Design")
        Ikin.setPosition(0.1, 0.2, 1.0)
        Ikin.getStructureMatrix(0.1, 0.2, 1.0)
        self.assertEqual(Ikin.getStructureMatrixMatrix(), AT)

    def test_Ikin_inversekinematics_compliance644(self):
        """
        Test for reproduction of results computed with an older version of WiPy
        here: inverse kinematics, reference data determined on 06.12.2017
        """        
        l = [2.5882812830138846, 2.433762519228201, 2.659924810967408, 2.8019992862240346, 2.5882812830138846, 2.433762519228201, 2.659924810967408, 2.8019992862240346]
        Irobot.applyModel("IPAnema1Design")
        Ikin.setPosition(0.1, 0.2, 1.0)
        self.assertEqual(Ikin.inverseKinematics2(), l)

    def test_Iws_cablecableinterference(self):
        Irobot.applyModel("IPAnema3Design")
        Iws.updateClippingPlanes()
        self.assertEqual(Iws.getMinInterferenceAngle(), 0.5188656962010074)
        Iws.setCableCableSetting(False)
        Iws.setEtaMax(0.4)
        Iws.setupCableCableInterferenceSets()

    def test_Iws_cableprintinterference(self):
        Irobot.applyModel("IPAnema3Design")
        Iws.updateClippingPlanes()
        Iws.setPn(0,0,-0.3)
        Iws.setPrintDir(0.6,1,0.6,-1)
        Iws.setEllipseExp(4)
        Iws.setPrintCenter(0,0,1)
        Iws.setPrintHeight(1)
        Iws.setNumberOfPrintLayers(24)
        Iws.setPrintParaVisNumber(2)
        Iws.calculatePrintShadowData()
        self.assertEqual(Iws.checkInterferenceWithPrintEllipseMultLayerCriteria(), True)
        self.assertEqual(Iws.checkInterferenceWithPrintParaMultLayerCriteria(), True)
        Iws.calculateInterferenceWithPrintEllipse()
        Iws.calculateInterferenceWithPrintParallelogram()
        Iws.calculateInterferenceWithPrintStarShape()

    def test_Iws_checkPoseForCableCableCollision(self):
        Irobot.applyModel("IPAnema3Design")
        self.assertEqual(Iws.checkPoseForCableCableCollision(0,0,1.5,-4.0,-2.0,0.0,0.6967067093471654,-0.7173560908995228,0,0.7173560908995228,0.6967067093471654,0,0,0,1), False)
        self.assertEqual(Iws.checkPoseForCableCableCollision(0,0,1.5,-4.0,-2.0,1.2000000000000002,0.6967067093471654,-0.7173560908995228,0,0.7173560908995228,0.6967067093471654,0,0,0,1), True)

if __name__ == '__main__':
    print VERSION_INFO

    suite = unittest.TestLoader().loadTestsFromTestCase(WiPyMethods)
    unittest.TextTestRunner(stream=sys.stdout, verbosity=2).run(suite)
    
    print VERSION_REMARK