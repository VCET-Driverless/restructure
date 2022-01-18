import os
import datetime


class Log:
    log_folder = "logs"
    test_count = 0
    today = datetime.datetime.now()

    if not os.path.isdir(log_folder):
        os.mkdir(log_folder)

    day_folder = log_folder + "/" + \
        str(today.day) + "-" + str(today.month) + "-" + str(today.year)

    if not os.path.isdir(day_folder):
        os.mkdir(day_folder)

    while os.path.isfile(day_folder + "/" + "test" + str(test_count) + ".json"):
        test_count = test_count + 1

    log_file_name = day_folder + "/" + "test" + str(test_count)

    f = open(log_file_name + ".json", "w+")

    def filename(self):
        return Log.f, Log.log_file_name
