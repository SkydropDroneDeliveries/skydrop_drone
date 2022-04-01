# Program to show various ways to read and
# write data in a file.

LOG_StartTime = 9

def print_logfile():

    for x in range(6):
        file1 = open("LOG.txt","a")
        L = ["-Log entry Time:" + str(LOG_StartTime) + "\n" ,
            "-Apartemtn ID \n"
            "-Navigation Phase  \n",
            "      -GPS Phase:  \n",
            "      -VO Phase:  \n",
            "      \n",
            "-Marker Identification \n",
            "      -Marker identified: \n",
            "      -Alignment done:  \n",
            "      -Distance Maintained to marker:  \n",
            "-Delivery time:  \n",
            "\n",
            "Delivery Success:  \n"     
            "\n",
            "=========================================\n",     
            "\n",
            "\n",
            "\n"
            ]

        file1.write("DELIVERY NUMBER: ")
        file1.write('%d' % x)
        file1.write("\n")
        file1.write("**************************************\n")

        file1.writelines(L)
        file1.close() 

    file1 = open("LOG.txt","r+")

    print("Log Output")
    print(file1.read())
    print()

if __name__ == '__main__':
    print_logfile()