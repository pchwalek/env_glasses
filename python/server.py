from influxdb import InfluxDBClient

database_name = 'airspec'

def start_server_logger(client):
    # client = InfluxDBClient(host='localhost', port=8086)

    # check if database already exists and if not, create it
    database_exist = False
    current_databases = client.get_list_database()
    for database in current_databases:
        if database == database_name:
            database_exist = True
            break
    if not database_exist:
        client.create_database(database_name)
    client.switch_database(database_name)

if __name__ == '__main__':
    client = InfluxDBClient(host='localhost', port=8086)
    start_server_logger(client)