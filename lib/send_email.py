
# -*- coding: utf-8 -*-
"""
Created 6/25

@author: Reid Simmons

Extensive use of chatGPT to figure out how to send Outlook emails with oauth2

API: send_email.send(sender, recipients, subject, text, images=[], inline=False)
     * sender: email address must match that in ../param/token_cache.json
     * recipients: comma-separated list of email addresses
     * subject: subject line
     * text: can be plain text or html
     * images: optional list of images (byte arrays, not file names)
     * inline: whether the images are inline, or attachments; if inline, refer to
               them in the text using, for instance, '<img src="cid:image1" />', 
               for image1, image2, ...
"""
import pathlib, base64, msal, requests

terrabot_dir = pathlib.Path(__file__).parent.parent
TOKEN_CACHE_FILE = terrabot_dir / 'param/token_cache.json'
CLIENT_ID = 'ff1cc03d-5766-430d-b45d-587116b60294'
AUTHORITY = f"https://login.microsoftonline.com/common"
SCOPES = ['User.Read', 'Mail.Send']

def init(reinitialize=False):
    print(TOKEN_CACHE_FILE)
    try:
        cache = msal.SerializableTokenCache()
        with open(TOKEN_CACHE_FILE, "r") as f:
            cache.deserialize(f.read())
    except Exception as e:
        print("Failed to find token cache:", e)
        return None
    
    app = msal.PublicClientApplication(
        CLIENT_ID,
        authority=AUTHORITY,
        token_cache = cache,
    )
    accounts = app.get_accounts()
    if accounts:
        token_result = app.acquire_token_silent(SCOPES, account=accounts[0])
    elif not reinitialize:
        print("Invalid cache")
        token_result = []
    else:
        print("Invalid cache. Initiating device flow login...")
        flow = app.initiate_device_flow(scopes=SCOPES)
        if "user_code" not in flow:
            raise ValueError("Failed to create device flow")

        print(f"\nGo to {flow['verification_uri']} and enter code: {flow['user_code']}")
        print("Waiting for authentication...\n")

        token_result = app.acquire_token_by_device_flow(flow)

    # === SAVE TOKEN CACHE ===
    if cache.has_state_changed:
        with open(TOKEN_CACHE_FILE, "w") as f:
            f.write(cache.serialize())

    if "access_token" in token_result:
        return token_result["access_token"]
    else: return None

def setCacheLocation(location):
    global TOKEN_CACHE_FILE
    TOKEN_CACHE_FILE = location

def msg_attachments(images, inline):
    return [{"@odata.type": "#microsoft.graph.fileAttachment",
            "name": "image%d" %(i+1),
            "contentType": "image/jpeg",
            "contentBytes": base64.b64encode(image).decode('utf-8'),
            "isInline": inline,
            "contentId": "image%d" %(i+1)
            } for i, image in enumerate(images)]

# Doesn't need passwd, but including to be consistent with older version
def send(from_address, passwd, to_addresses, subject, text, images=[], inline=False):
    try:
        access_token = init()
        if (not access_token): return False

        headers = { 'Authorization': f'Bearer {access_token}',
                    'Content-Type': 'application/json'}
        user_info = requests.get('https://graph.microsoft.com/v1.0/me', headers=headers).json()
        token_address = user_info.get("userPrincipalName").lower()
        if (token_address != from_address.lower()):
            raise Exception("from address %s does not match token address %s"
                            %(from_address, token_address))

        to_addresses = [{"emailAddress": {"address": address.strip()}}
                        for address in to_addresses.split(',')]
        content_type = "html" if ('<' in text and '</' in text) else "text"
        message = {
            "message": {
                "toRecipients": to_addresses,
                "subject": subject,
                "body": {"contentType": content_type, "content": text},
                "attachments": msg_attachments(images, inline),
            },
        "saveToSentItems": "true"
        }
        response = requests.post('https://graph.microsoft.com/v1.0/me/sendMail',
                                  headers=headers, json=message).status_code
        if response != 202: 
            print('Failed to send with rsponse %d' %response)
        return response == 202
    except Exception as e:
        print('Failed to send:', e)
        return False

# Here's a simple example (please don't actually use it as is, since it will spam me)
if __name__ == "__main__":
    #init(True)
    images = []
    for file_name in ["../simulator.JPG", "../system_diagram.jpg"]:
        with open(file_name, 'rb') as f: images += [f.read()]
    if send("terrabot0@outlook.com", "Simmons482", "reidgs@hotmail.com, reids@cs.cmu.edu", 
            "Hello", '<b>This is a test</b><p><img src="cid:image1" /><p><img src="cid:image2" />', 
            images, inline=True):
        print("Successfully sent!")
