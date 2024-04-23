#-------------------------------------------------------------------------------
# Name:        Generate input for the WireX documentation in LateX
# Purpose:
#
# Author:      asp
#
# Created:     14.04.2014
# Copyright:   (c) asp 2014
# Licence:     <your licence>
#-------------------------------------------------------------------------------
#!/usr/bin/env python

hasWiPy=False
hasIWC=False

try:
    import WiPy
    hasWiPy = True
    
except:
    print "No need to load WiPy. Running in embedded mode"

try:
    from IWC import *
    hasIWC = True
except:
    print "Did not load WireCenter standard interface"

import inspect

##    print "\\begin{table}"
##    print "\caption{List of all entries of the module \\texttt{", mod.__name__,"}}"
##    print "\\begin{tabular}{lp{5cm}}"
##    print "\hline function name & documentation\\\\"
##    print "\hline"
##    for (x,b) in inspect.getmembers(mod):
##        #print a,b
##        print "\\texttt{", x, "}&", inspect.getdoc(b), "\\\\"
##
##    print "\hline"
##    print "\end{tabular}"
##    print "\end{table}"

def main():
    modules = []
    if hasWiPy | hasIWC:
        modules.append(Ikin)
        modules.append(Iws)
        modules.append(Irobot)
        modules.append(Icontrol)
    if hasIWC:
        modules.append(Iscene)
        modules.append(Iapp)
        modules.append(Iplugin)
    print modules
    
    for mod in modules:
        print "\subsection{Module", mod.__name__,"}"
        for (x,b) in inspect.getmembers(mod):
            if ('__' in x):
                continue
            
            str =  "\\texttt{"+mod.__name__+"."+x+"()}: "+inspect.getdoc(b)
            # before writing the python help string we do python encoding escaping spectial characters
#            str = str.replace('\\',"$\\backslash$")
            str = str.replace('_','\_')
            str = str.replace('^','\^{}')
            str = str.replace('%','\%')
            str = str.replace('#','\#')
            str = str.replace('$','\$')
            str = str.replace(' | ',' $|$ ')
            str = str.replace('||','$||$')
            str = str.replace('~','\~')
            str = str.replace('>','$>$')
            str = str.replace('<','$<$')
#            str = str.replace('{','\{')
#            str = str.replace('}','\}')
            str = str.replace('&','\&')
            print str
            print ""

    pass

if __name__ == '__main__':
    main()
