import re

'''
优化yml文件的格式以方便解析
'''

# Define the regex pattern and replacement
pattern = r'descriptor:\s*"([\d\s]+)"\s*'
replacement = r'descriptor:"\1"'

# Read the file
with open('./assets/ch11-vocabulary-regex.yml', 'r', encoding='utf-8') as file:
    content = file.read()

# Apply the regex substitution
modified_content = re.sub(pattern, replacement, content)

# Write the modified content back to the file
with open('./assets/ch11-vocabulary-regex.yml', 'w', encoding='utf-8') as file:
    file.write(modified_content)

print("File has been updated successfully.")