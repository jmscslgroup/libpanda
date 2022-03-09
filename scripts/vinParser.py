import requests
import json
##read the local VIN from file
vin = open("/etc/libpanda.d/vin","r").readline().strip()
#need internet connection to work
url = 'https://vpic.nhtsa.dot.gov/api/vehicles/decodevinvalues/' + vin + '?format=json'

try:
    vinDataRequest = requests.get(url)
except requests.Timeout:
    print("nhtsa: connection timed out")
except requests.ConnectionError:
    print("nhtsa: connection failed")

vinData = vinDataRequest.json()
results = vinData['Results'][0]
#save the make, model, trim, and model year
vinDetails = {'Make':results['Make'].encode('utf-8'),'Model':results['Model'].encode('utf-8'),'Trim':results['Trim'].encode('utf-8'), 'Year':results['ModelYear'].encode('utf-8')}
dest_file = '/etc/libpanda.d/vin_details.json'
output_file = open(dest_file, 'w')
json.dump(vinDetails, output_file)
