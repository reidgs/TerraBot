# -*- coding: utf-8 -*-
"""
Created on Sun Aug 27 23:49:21 2023

@author: thomas cantalapiedra
"""

import smtplib
from email.message import EmailMessage

def init(sender, password):
    port = 587
    host = "smtp.office365.com"
    timeout = 2.5 # Seconds
    server = smtplib.SMTP(host, port, timeout=timeout) 
    server.ehlo()
    server.starttls()
    server.login(sender, password)
    return server

"""
****************************** PARAMETERS ***********************************
* from_address[string]: the outlook account assigned to your group
* password[string]: the password associated with the assigned outlook account
* to_address[string]: email addresses separated by ', ' (COMMA + SPACE)
* subject[string]: the subject of the email
* text[string]: the content of the email (sensor readings, actuator values,
#               explanations, etc.)
* images[list of jpgs]: the image to be included
*****************************************************************************
"""
def send(from_address, password, to_address, subject, text, images=[]):

    try:
        server = init(from_address, password)
    except Exception as e:
        print("Failed to send: %s" %e)
        return False

    msg = EmailMessage()
    msg['Subject'] = subject
    msg['From'] = from_address
    msg['To'] = to_address
    msg.set_content(text)
    for image in images:
        msg.add_attachment(image, maintype='image', subtype='jpeg')

    success = True
    try:
        server.sendmail(from_address, to_address.split(','), msg.as_string())
    except Exception as e:
        print("Failed to send: %s" %e)
        success = False

    server.close()
    return success

