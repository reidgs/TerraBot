# -*- coding: utf-8 -*-
"""
Created 10/24

@author: Reid Simmons

Prior outlook email stoppoed working - needs oauth2
Tried gmail - tokens needed to be refreshed regularly; now trying Twillo
  (which previous TerraBot team used successfully)
"""
import os, base64
from sendgrid import SendGridAPIClient
from sendgrid.helpers.mail import Mail, Attachment

def init():
    try:
        credential_path = os.path.join(os.path.expanduser('~'),
                                       '.credentials/twillo.key')
        with open(credential_path, "r") as f:
            api_key = f.readline()
    except Exception as e:
        print("Failed to find api-key:", e)
        api_key = None
    return api_key

def add_attachments(msg, images):
    attachments = []
    for i, image in enumerate(images):
        encoded_image = base64.b64encode(image).decode()
        attachment = Attachment(file_content=encoded_image,
                                file_type='image/jpeg',
                                file_name='image %d' %(i+1),
                                disposition='attachment')
        attachments.append(attachment)
    attachments.reverse()
    msg.attachment = attachments
    
def send(from_address, password, to_addresses, subject, text, images=[]):
    try:
        api_key = init()
        if (not api_key): return False
        to_addresses = [address.strip() for address in to_addresses.split(',')]
        text_content = html_content = None
        if  ('<' in text and '/>' in text): html_content = text
        else: text_content = text
        msg = Mail(from_email=from_address, to_emails=to_addresses,
                   subject=subject,
                   plain_text_content=text_content, html_content=html_content)
        if (len(images) > 0): add_attachments(msg, images)
        sg = SendGridAPIClient(api_key)
        response = sg.send(msg)
        return response.status_code == 202
    except Exception as e:
        print('Failed to send:', e)
        return False
