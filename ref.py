import sys
class Test:
    def __init__(self):
        print("new instance")
    def set(self,val):
        self.val = val
    def __del__(self):
        print("del")

def fun():
    ins = Test()
    ins.val = 3
    print("fun(): ref count: "+str(sys.getrefcount(ins)))
    return ins
def fun1(var):
    #print("fun1(): ref count: "+str(sys.getrefcount(var)))
    print(sys.getrefcount(var))
    return

this_ins = fun()
#print("main(): ref count: "+str(sys.getrefcount(this_ins)))
fun1(this_ins)


new_ins = Test()
print("main(): ref count: "+str(sys.getrefcount(new_ins)))

