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
Created on Fri Sep 24 12:42:37 2017

@author: Andreas Pott

This is an experimental baseline do unit tests inside WireCenter 
"""

import unittest
import os
from IWC import *

class WiPyMethods(unittest.TestCase):
    """ 
    test the WiPy module; test must start with "test_" to be autoexecuted by the 
    framework
    """

    def test_Wc_Iapp_Roi(self):
        self.assertEqual(Iapp.setRoi(-1,-1,-1,1,1,1),None)

def main():    
    suite = unittest.TestLoader().loadTestsFromTestCase(WiPyMethods)
    unittest.TextTestRunner(verbosity=2).run(suite)