import edgeimpulse as ei
import os

ei.API_KEY = "ei_ae7962939fff88e0a205069eca7afe9e09793314bf00be79"

def download_all_samples():
    print("Listing samples...")
    try:
        all_samples = []
        for category in ["training", "testing"]:
            print(f"Fetching {category} sample IDs...")
            try:
                infos = ei.experimental.data.get_sample_ids(category=category)
                all_samples.extend([(info.sample_id, category) for info in infos])
            except Exception as e:
                print(f"Could not fetch {category} IDs: {e}")
        
        print(f"Total samples found: {len(all_samples)}")
        os.makedirs("data", exist_ok=True)
        
        for sample_id, category in all_samples:
            target_dir = os.path.join("data", category)
            os.makedirs(target_dir, exist_ok=True)
            
            try:
                sample = ei.experimental.data.download_samples_by_ids([sample_id])[0]
                # Use sample_id in filename to prevent overwriting
                extension = os.path.splitext(sample.filename)[1]
                save_name = f"{os.path.splitext(sample.filename)[0]}.{sample_id}{extension}"
                filename = os.path.join(target_dir, save_name)
                
                # Check if sample.data has .read() or if it's already bytes/str
                if hasattr(sample.data, 'read'):
                    data = sample.data.read()
                else:
                    data = sample.data
                
                # If it's a string (likely for JSON), encode it
                if isinstance(data, str):
                    data = data.encode('utf-8')
                    
                with open(filename, "wb") as f:
                    f.write(data)
                print(f"Downloaded: {filename}")
            except Exception as e:
                print(f"Failed to download sample {sample_id}: {e}")

    except Exception as e:
        print(f"General error: {e}")

if __name__ == "__main__":
    download_all_samples()
