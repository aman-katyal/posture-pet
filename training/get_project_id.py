import requests

API_KEY = "ei_ae7962939fff88e0a205069eca7afe9e09793314bf00be79"

url = "https://studio.edgeimpulse.com/v1/api/projects"
headers = {
    "x-api-key": API_KEY,
    "Accept": "application/json"
}

response = requests.get(url, headers=headers)
if response.status_code == 200:
    data = response.json()
    if data['success']:
        for p in data['projects']:
            print(f"Found Project: {p['name']} (ID: {p['id']})")
    else:
        print(f"Error: {data['error']}")
else:
    print(f"Failed to fetch projects: {response.status_code} - {response.text}")
