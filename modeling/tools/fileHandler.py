import os 
import sys 
import json 
import csv 
import pandas as pd 

class fileHandler(object):
    """
    fileHandler

    it is a class that allows you to look for specific file formats, 
    write and read the formats related to the point cloud and MS images.
    """

    def get_FileList(self, Fformat, path2folder):
        """
        INPUT:
            Fformat: string that cotain the format of the need it file, ex. [obgem, jpeg, tiff, pdf]
            path2folder: string that have the path to the folder where must be look for the files
        OUTPUT:
            list of strings with the name of the files 
        """
        # Init var 
        lst2ret = []
        # Get the len of the string to substract that from the end of the path2folder string
        lenFormat = len(Fformat)
        # Get the list of the files 
        rawList = os.listdir( path2folder )
        # Walk over the raw file list 
        for i in rawList:
            # General string
            gPath = path2folder+"/"+i
            # Verify if it is a file or folder 
            if(os.path.isfile(gPath) == True):
                # Verify that string length of the file/folder name is bigger that the 
                # defined format -- Prevent bad index  
                if(len(i) > lenFormat):
                    # Verify the extension 
                    fileExt = i[ len(i)-lenFormat:len(i) ]
                    # Append if the file have a related format -- string verification not bit-by-bit
                    if( fileExt == Fformat ):
                        lst2ret.append( i )
        return lst2ret