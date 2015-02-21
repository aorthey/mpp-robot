def folderNameFromHvalues(h1step,h2step,h3step):
        ###############################################################################
        #### create folder name from h1,h2,h3 step sizes:
        ###############################################################################
        hfolder  = "/h3_"
        hfolder += str(int(h3step))
        hfolder += "_"
        hfolder += str(h3step-int(h3step))[2:]
        hfolder += "_h2_"
        hfolder += str(int(h2step))
        hfolder += "_"
        hfolder += str(h2step-int(h2step))[2:]
        hfolder += "_h1_"
        hfolder += str(int(h1step))
        hfolder += "_"
        hfolder += str(h1step-int(h1step))[2:]
        ###############################################################################
        return hfolder

