class bcolors:
    TITLE = '\x1b[6;30;42m'
    GC = '\x1b[5;30;43m'
    DC = '\x1b[6;30;41m'
    NC = '\x1b[0;30;47m'
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    VOL = '\x1b[6;31;47m'


def printC(str,color = 'title'):
    if(color == 'title'):
        print(bcolors.TITLE + str + bcolors.ENDC)
    elif(color == 'blue'):
        print(bcolors.OKBLUE + str + bcolors.ENDC)
    elif(color == 'cyan'):
        print(bcolors.OKCYAN + str + bcolors.ENDC)
    elif(color == 'green'):
        print(bcolors.OKGREEN + str + bcolors.ENDC)
    elif(color == 'underline'):
        print(bcolors.UNDERLINE + str + bcolors.ENDC)
    elif(color == 'warning'):
        print(bcolors.WARNING + str + bcolors.ENDC)
    else:
        print(str)