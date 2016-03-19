from wpilib.command import Command


# def killParents(cmd):
def wrapWithOrphan(cmd):
    if OrphanCommand not in cmd.__bases__:
        if Command in cmd.__bases__:
            cmd.__bases__ = filter(lambda x: x is not Command, cmd.__bases__)  # I feel kind of dirty
        cls = cmd.__class__
        cmd.__class__ = cls.__class__(cls.__name__ + "Orphan", (cls, OrphanCommand), {})


class OrphanCommand(Command):
    """
    A Command that can be moved around between parents.
    """
    def setParent(self, parent):
        if parent is None:
            self.parent = None
            self.locked = False
        else:
            super().setParent(parent)
