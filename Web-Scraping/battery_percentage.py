import sys
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from playsound import playsound
import datetime
import time
import re

def get_battery_percentage():
    chrome_options = Options()
    chrome_options.binary_location = '/usr/bin/google-chrome-stable'
    chrome_options.add_argument('--headless')
    chrome_options.add_argument('--disable-extensions')
    
    driver = webdriver.Chrome(options=chrome_options)
    
    url = 'http://192.168.12.1/bms'
    driver.get(url)
    
    time.sleep(5)
    
    page_source = driver.page_source
    html_content = str(page_source)
    
    soc_index = html_content.find("SOC：")
    
    if soc_index != -1:
        match = re.search(r'\d+', html_content[soc_index:])
        if match:
            soc_value = match.group()
            driver.quit()
            return int(soc_value)
    
    driver.quit()
    return None

def main(loop=True):
    last_min = -1

    if loop:
        print("[INFO] Running check on battery status continuously.")
    else:
        print("[INFO] Running check on battery status.")

        max_retries = 5
        retries = 0
        while retries < max_retries:
            try:
                battery_percentage = get_battery_percentage()
                if battery_percentage is not None:
                    print("[INFO] Battery percentage:", battery_percentage)
                    if battery_percentage < 25:
                        playsound('warning.mp3')
                    break
            except Exception as e:
                print("[ERROR] An error occurred:", str(e))
                print(f"[INFO] Retrying ({retries + 1}/{max_retries}) in 5 seconds...")
                time.sleep(5)
                retries += 1
        else:
            print(f"[ERROR] Unable to retrieve battery percentage after {max_retries} retries. Please make sure the robot is ON and your device is connected to its Wi-Fi.")

    while loop:
        if datetime.datetime.now().minute != last_min:
            try:
                battery_percentage = get_battery_percentage()
                if battery_percentage is not None:
                    print("[INFO] Battery percentage:", battery_percentage)
                    if battery_percentage < 25:
                        playsound('warning.mp3')
                    continue
            except Exception as e:
                print("[ERROR] An error occurred:", str(e))
                print(f"[INFO] Retrying in 5 seconds...")
                time.sleep(5)

            last_min = datetime.datetime.now().minute

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1].lower() == 'noloop':
        main(loop=False)
    else:
        main()
