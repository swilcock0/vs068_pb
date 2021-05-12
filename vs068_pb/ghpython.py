import numpy as np

# import rhinoscriptsyntax as rs
# from compas.rpc import Proxy

## pythonw prevents a window from opening
# with Proxy('vs068_pb.ghpython', python='C:\Users\Sam\Anaconda3\envs\project\pythonw.exe') as proxy:

#     proxy.run_test()

def run_test():
    import sys
    print("Test", file=sys.stdout)

def load_reach_data():
    import vs068_pb.reachability as reach
    test = reach.Reachability()
    test.load_data()
    test.view_data()
    return test.data