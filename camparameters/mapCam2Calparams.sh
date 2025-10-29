cd ~/Documents/Surround-View-master/campar/

# Step 1: Backup originals
mkdir backup
cp Camparam*.yaml backup/

# Step 2: Create temporary files with correct mapping
cp backup/Camparam2.yaml Camparam0_new.yaml  # Front
cp backup/Camparam3.yaml Camparam1_new.yaml  # Left
cp backup/Camparam0.yaml Camparam2_new.yaml  # Right
cp backup/Camparam4.yaml Camparam3_new.yaml  # Rear

# Step 3: Replace old files
mv Camparam0_new.yaml Camparam0.yaml
mv Camparam1_new.yaml Camparam1.yaml
mv Camparam2_new.yaml Camparam2.yaml
mv Camparam3_new.yaml Camparam3.yaml

# Verify
ls -la Camparam*.yaml

