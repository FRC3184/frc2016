import wpilib

class Command:
    def __init__(self):
        self.requirements = []
    def require(self, subsystem):
        self.requirements.append(subsystem)
    def start(self):
        pass
    def run(self):
        pass
    def finish(self):
        pass
    def isFinished(self):
        return False
class Subsystem:
    def __init__(self):
        self.locked = False
    def isLocked(self):
        return self.locked
    def lock(self):
        self.locked = True
    def unlock(self):
        self.locked = False
    
class CommandBasedRobot(wpilib.IterativeRobot):
    def __init__(self):
        super().__init__()
        self.commands = []
        self.newCommands = []
        self.subsystems = {}
        
    def registerCommand(self, cmd):
        for requirement in cmd.requirements:
            if requirement in self.subsystems.keys():
                subsystem = self.subsystems[requirement]
                if subsystem.isLocked():
                    print("Subsystem \"{}\" being used! Cannot register command!".format(requirement))
                    return None
                else:
                    subsystem.lock()
            else:
                print("Cannot find subsystem \"{}\". This may lead to errors later.".format(requirement))
        self.newCommands.append(cmd)
    def unregisterCommand(self, cmd):
        self.commands.remove(cmd)
        cmd.finish()
        
        for requirement in cmd.requirements:
            if requirement in self.subsystems.keys():
                subsystem = self.subsystems[requirement]
                subsystem.unlock()
            else:
                print("Cannot find subsystem \"{}\". This may lead to errors later.".format(requirement))
    def runSchedule(self):
        for cmd in self.newCommands:
            cmd.start()
        self.commands += self.newCommands
        newCommands = []
        for cmd in self.commands:
            cmd.run()
            if cmd.isFinished():
                unregisterCommand(cmd)

    def teleopPeriodic(self):
        self.runSchedule()
        
    def autonomousPeriodic(self):
        self.runSchedule()
