import datetime
import sys


class DataLogger:
    def __init__(self, logfile=sys.stdout, curtime=datetime.datetime.now):
        self.logfile = logfile
        self.curtime = curtime
        self.dataSources = {}
        self.isReady = False

    def close(self):
        self.isReady = False
        self.logfile.close()

    def add_data_source(self, name, source):
        self.dataSources[name] = source

    def ready(self):
        self.isReady = True
        print("[Status] Time", file=self.logfile, end="")
        for k, _ in self.dataSources:
            print(", {}".format(k), end="", file=self.logfile)
        print(file=self.logfile)

    def log_data(self):
        if not self.isReady:
            return None
        print("[Data] {}".format(self.curtime()), end="", file=self.logfile)
        for _, v in self.dataSources:
            print(", {}".format(v()), end="", file=self.logfile)
        print(file=self.logfile)

    def event(self, text):
        print("[Event] {}, {}".format(self.curtime(), text))
