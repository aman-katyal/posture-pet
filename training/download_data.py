import requests
import zipfile
import os
import time

API_KEY = "ei_ae7962939fff88e0a205069eca7afe9e09793314bf00be79"
PROJECT_ID = "966723"
BASE_URL = "https://studio.edgeimpulse.com/v1/api"

headers = {
    "x-api-key": API_KEY,
    "Accept": "application/json"
}

def trigger_export():
    print("Triggering project export...")
    url = f"{BASE_URL}/{PROJECT_ID}/jobs/export/original"
    response = requests.post(url, headers=headers)
    if response.status_code == 200:
        job_id = response.json().get("jobId")
        print(f"Export job started. Job ID: {job_id}")
        return job_id
    else:
        print(f"Failed to trigger export: {response.text}")
        return None

def wait_for_job(job_id):
    print("Waiting for job to complete...")
    url = f"{BASE_URL}/{PROJECT_ID}/jobs/{job_id}/status"
    while True:
        response = requests.get(url, headers=headers)
        if response.status_code == 200:
            status = response.json().get("job", {}).get("finished")
            if status:
                print("Job finished!")
                return True
            else:
                print("Job still running...")
        else:
            print(f"Error checking job status: {response.text}")
            return False
        time.sleep(5)

def get_download_link():
    print("Fetching download link...")
    url = f"{BASE_URL}/{PROJECT_ID}/downloads"
    response = requests.get(url, headers=headers)
    if response.status_code == 200:
        downloads = response.json().get("downloads", [])
        # Find the latest export
        for d in downloads:
            if "/api/v1/raw-data/export" in d.get("link", ""):
                return d.get("link")
    print("Download link not found.")
    return None

def download_file(link):
    url = f"https://studio.edgeimpulse.com{link}"
    print(f"Downloading from {url}...")
    response = requests.get(url, headers=headers, stream=True)
    if response.status_code == 200:
        with open("data.zip", "wb") as f:
            for chunk in response.iter_content(chunk_size=8192):
                f.write(chunk)
        print("Download complete.")
        return True
    else:
        print(f"Failed to download: {response.status_code}")
        return False

def main():
    job_id = trigger_export()
    if not job_id:
        # Check if a download already exists if trigger failed
        link = get_download_link()
    else:
        if wait_for_job(job_id):
            link = get_download_link()
        else:
            link = None

    if link:
        if download_file(link):
            print("Extracting...")
            os.makedirs("data", exist_ok=True)
            with zipfile.ZipFile("data.zip", "r") as zip_ref:
                zip_ref.extractall("data")
            print("Extraction complete.")
    else:
        print("Could not get download link.")

if __name__ == "__main__":
    main()
