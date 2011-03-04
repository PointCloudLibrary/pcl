#!/usr/bin/python
'''
Created on 13.12.2009

A simple script to clean up cmake-generated dot files for dependency analysis.

This program is free software; you can redistribute it
and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation;
either version 2, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

@author: Johannes Wienke <languitar at semipol dot de>
'''

from optparse import OptionParser
import logging

logger = logging.getLogger(__name__)

class Processor:
    
    def __init__(self, filename, outfile):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.filename = filename
        self.outfile = outfile
        
        self.removeUsrLib = False
        self.additionalPatterns = []
        
        self.logger.debug("Parser set up successfully")
        
    def isRemoveUsrLib(self):
        return self.removeUsrLib
    
    def setRemoveUsrLib(self, remove):
        self.removeUsrLib = remove
        
    def addPatternToRemove(self, pattern):
        self.additionalPatterns.append(pattern)
        
    def process(self):
        
        self.logger.debug("Tyring to open file '" + self.filename + "'")
        inputFile = open(self.filename, 'r')
        lines = inputFile.readlines()
        inputFile.close()
        
        self.logger.debug("read lines:" + str(lines))
        
        self.logger.debug("starting filtering")
        
        if self.removeUsrLib:
            lines = self.doRemoveByContains(lines, "/usr/lib")
            
        for pattern in self.additionalPatterns:
            lines = self.doRemoveByContains(lines, pattern)
            
        self.logger.debug("finished filtering")
        
        self.logger.debug("writing output to " + self.outfile)
        outputFile = open(self.outfile, 'w')
        for line in lines:
            outputFile.write(line)
        outputFile.close()
    
    def doRemoveByContains(self, lines, pattern):
        self.logger.info("Removing nodes containing " + pattern)
        
        # first, find the nodes to remove in the top definition of node labels
        # and already remove them there
        self.logger.debug("Searching for node definitions containing " + pattern)
        linesWithoutLabels = []
        removedNodes = []
        for line in lines:
            
            # skip linking lines right now
            if line.find("->") >= 0:
                linesWithoutLabels.append(line)
                continue
            if line.find(pattern) >= 0:
                trimmed = line.strip()
                parts = trimmed.split()
                if len(parts) >= 2:
                    removedNodes.append(parts[0])
                    self.logger.debug("Removing line: " + line)
                else:
                    linesWithoutLabels.append(line)
            else:
                linesWithoutLabels.append(line)
                
        # now also remove all links with the filtered nodes
        self.logger.debug("Removing links to filtered nodes")
        filteredLines = []
        for line in linesWithoutLabels:
            
            filterLine = False
            for node in removedNodes:
                if line.find(node) >= 0:
                    filterLine = True
                    break;
                
            if not filterLine:
                filteredLines.append(line)
            else:
                self.logger.debug("Filtering link: " + line)
        
        return filteredLines
    
    removeUsrLib = property(fget=isRemoveUsrLib, fset=setRemoveUsrLib)

def main():
    
    logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s %(levelname)-8s [%(name)s] %(message)s')
    
    logger.debug("configuring option parser")
    
    usage = "usage: %prog [options] <input dot source>"
    parser = OptionParser(usage)
    parser.add_option("-f", "--file", dest = "filename",
                      help = "write processed dot graph source to FILE (default = cleaned.dot)",
                      metavar = "FILE", default = "cleaned.dot")
    parser.add_option("-u", "--removeusrlib", dest="removeUsrLib",
                      action = "store_true",
                      help = "remove targets located in /usr/lib",
                      default = "false")
    parser.add_option("-p", "--pattern", dest="patterns",
                      action = "append",
                      help = "adds an additional pattern to remove nodes with",
                      default = [])
    
    logger.debug("parsing command line")
    
    (options, args) = parser.parse_args()
    
    if len(args) != 1:
        logger.debug("Required arguments missing")
        parser.error("incorrect number of arguments. Need source file as argument.")
        exit(1)
        
    logger.debug("Instantiating processor object")
        
    processor = Processor(args[0], options.filename)
    
    processor.removeUsrLib = options.removeUsrLib
    for pattern in options.patterns:
        processor.addPatternToRemove(pattern)
    
    processor.process()

if __name__ == '__main__':
    main()
