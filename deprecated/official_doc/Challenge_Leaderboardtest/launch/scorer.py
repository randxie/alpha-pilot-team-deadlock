#!/usr/bin/env python

import yaml
import sys
import glob, os

R=10

class Scorer():
  def __init__ (self, directory):
     self.directory = directory
     self.scores = {}
     self.processFiles()
     self.writeScores()     

  def processFiles(self):
     run = 1
     for f in os.listdir(self.directory):
        if f.endswith(".yaml"):
           stream = file(os.path.join(self.directory,f), 'r')
           data = yaml.load(stream)
           if data["Result"]=="Challenge Completed":
              n = len(data.keys())-1
              N = 0
              for i in range(n):
                 if data["Gate%d"%i]["Success"]=="True":
                    N+=1
                    T = data["Gate%d"%(n-1)]["Time"]
              self.scores["Run%02d"%run] = (N*R-T)
           else:
              self.scores["Run%02d"%run] = 0
        stream.close()
        run+=1

  def writeScores(self):
     stream=file('scores.yaml', 'w')
     yaml.dump(self.scores, stream, default_flow_style=False)
     stream.close()


if __name__ == '__main__':
   Scorer(sys.argv[1])
