import os
import traceback
import sqlite3
import traceback




FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)
FILE_PATH_FOR_DATABASE = FILE_PATH.split(FILE_NAME)[0]+"datastream_robot.db"

MAX_DATA_COUNT = 50 #max of rows alowed in a table





def init_database_robot():
    """
    This function delete a previous database then create and initalize an empty database.
    There is currently 6 tables initialized :            
            r_aruco
            
    You can add more following the gloabl pattern.
    """

    #Delete previous one if it exists
    if os.path.isfile(FILE_PATH_FOR_DATABASE):
        os.remove(FILE_PATH_FOR_DATABASE)

    #Connect to database and close it when <with> block is exited
    with sqlite3.connect(FILE_PATH_FOR_DATABASE) as connection:
        cursor = connection.cursor()

        try :
            #Create these tables if they dont exist
            cursor.execute(''' CREATE TABLE IF NOT EXISTS r_aruco
                        (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        tag INTEGER,
                        ax INTEGER,
                        ay INTEGER,
                        bx INTEGER,
                        by INTEGER,
                        cx INTEGER,
                        cy INTEGER,
                        dx INTEGER,
                        dy INTEGER
                        )
                        ''')
            connection.commit()


        except sqlite3.Error as e:
            traceback_str = traceback.format_exc()
            print(f"Log [{os.times().elapsed}] - {FILE_NAME} : {e}\n{traceback_str}")




def insertData(table_name:str, data:dict):
    """
    Insert data into its appropriate table.

        table_name (str)    ->      table name.\n
        data (dict)         ->      data to insert with key as column and value as data.
                                    values must match accepted SQL types format.
    
    Return function success.
    """
    success = False

    #Get keys
    keys = ','.join(data.keys())
    #Get question marks based on number of data entries
    question_marks = ','.join(list("?"*len(data)))
    #Get values
    values = tuple(data.values())

    #Connect to database
    with sqlite3.connect(FILE_PATH_FOR_DATABASE) as connection:
        cursor = connection.cursor()

        try:

            #Count the number of row already in the table
            cursor.execute(f"SELECT COUNT(*) FROM {table_name}")
            nb_lines = cursor.fetchone()[0]

            #Delete exceeding lines
            if nb_lines >= MAX_DATA_COUNT:
                rows_to_delete = nb_lines - MAX_DATA_COUNT + 1
                cursor.execute(f"DELETE FROM {table_name} WHERE id IN (SELECT id FROM {table_name} ORDER BY id LIMIT ?)", (rows_to_delete,))

            #Inserting data into the table
            cursor.execute(f"INSERT INTO {table_name} ({keys}) VALUES ({question_marks})", values)
            connection.commit()
            success = True
        
        except sqlite3.Error as e:
            traceback.print_exc()
            print(f"Log [{os.times().elapsed}] - {FILE_NAME} : {e}")
    
    return success


if __name__ == "__main__":
    init_database_robot()