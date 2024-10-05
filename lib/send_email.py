# -*- coding: utf-8 -*-
"""
Created 10/24

@author: Reid Simmons

Prior outlook email stoppoed working - needs oauth2
This is a stopgap measure using gmail
"""
import httplib2, os, oauth2client, base64, apiclient
from oauth2client import client, tools, file
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from email.mime.base import MIMEBase
from email.message import EmailMessage

def get_credentials():
    scopes = "https://www.googleapis.com/auth/gmail.send"
    home_dir = os.path.expanduser('~')
    secret_file = os.path.join(home_dir, "client_secret.json")
    app_name = "TerraBot"

    credential_dir = os.path.join(home_dir, '.credentials')
    if not os.path.exists(credential_dir): os.makedirs(credential_dir)
    credential_path = os.path.join(credential_dir,
                                   "gmail-python-email-send.json")
    store = oauth2client.file.Storage(credential_path)
    credentials = store.get()
    if not credentials or credentials.invalid:
        flow = client.flow_from_clientsecrets(secret_file, scopes)
        flow.user_agent = app_name
        credentials = tools.run_flow(flow, store)
        print('Storing credentials to', credential_path)
    return credentials

def init():
    http = get_credentials().authorize(httplib2.Http())
    return apiclient.discovery.build('gmail', 'v1', http=http)
    
def send(from_address, password, to_address, subject, text, images=[]):
    try:
        service = init()
    except Exception as e:
        print("Failed to connect to server:", e)
        return False

    msg = EmailMessage()
    msg['Subject'] = subject
    msg['From'] = from_address
    msg['To'] = to_address
    msg.set_content(text, 'plain')
    for image in images:
        msg.add_attachment(image, maintype='image', subtype='jpeg')
    email = {'raw': base64.urlsafe_b64encode(msg.as_bytes()).decode()}
    try:
        message = service.users().messages().send(userId=from_address,
                                                  body=email).execute()
        return message
    except apiclient.errors.HttpError as error:
        print('Failed to send:', error)
