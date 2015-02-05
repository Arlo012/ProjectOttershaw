class PrimaryController:
    """
    Core control for balancing algorithm.
    Note: This is a singleton class (only one instance may exist)
    See documentation at: http://python-3-patterns-idioms-test.readthedocs.org/en/latest/Singleton.html
    """
    class __PrimaryController:
        def __init__(self, arg):
            self.val = arg
        def __str__(self):
            return repr(self) + self.val
    
    #Instance of this class    
    instance = None
    
    def __init__(self, arg):
        if not PrimaryController.instance:
            PrimaryController.instance = PrimaryController.__PrimaryController(arg)
        else:
            PrimaryController.instance.val = arg
            
    def __getattr__(self, name):
        return getattr(self.instance, name)
 
 
#Test code below to demonstrate below:
   
#a = PrimaryController("test")
#print(a)