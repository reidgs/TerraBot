# Installing TerraBot on Apple Silicon (M1)

## Convert the OVA to QCow2
- **Note:** If you have the Autonomous Agents USB stick, this part has already been done for you
- On Ubuntu: 
  ```
  tar -xvf TerraBot\ 2023.ova
  sudo apt-get install qemu-utils
  qemu-img convert -O qcow2 TerraBot\ 2023-disk001.vmdk TerraBot\ 2023.qcow2
  ```
  (For Mac, use `brew install qemu`)

  To check for consistency: `qemu-img check TerraBot\ 2023.qcow2`
  
## Create the Virtual Machine using UTM
Download UTM from ```https://mac.getutm.app```
(you can also get the exact same app on the app store but it will be $10 instead of free)

Open UTM and click **Create new Virtual Machine**, then click **Emulate**, followed by **Custom**

Checkmark **Skip ISO boot**

Keep clicking **continue** until Wizard is done

Click on **VM settings**, go to **drives** tab and click **New**.  Click **Import** and select the qcow2 drive.  Delete the other drive that exists.

Go to **QEMU** tab and disable **UEFI boot**

Go to **systems** tab and set CPU cores to total cores of the laptop (8 for basic M1).

Set RAM to 4GB (set it to 8+ if you have 16GB RAM on your machine)

Enable **Force Multicore**

Click **Save**
