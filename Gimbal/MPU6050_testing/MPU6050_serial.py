from data_time import get_data_time          #Contains the file which initialises the connection and collects and cleans data received from terminal

data_points = 1000


while True:    #Creates an infinite loop
    data_choice = input("Y for getting data list")
    if data_choice == "y":
        for i in range(data_points):
            print(get_data_time())