import smtplib

def init(sender, password):
    port = 587
    host = "smtp.gmail.com"
    server = smtplib.SMTP(host, port)
    server.ehlo()
    server.starttls()
    server.login(sender, password)
    return server

def send(from_address, password, to_address, subject, text):
    server = init(from_address, password)
    msg = ("From: %s\r\nTo: %s\r\nSubject: %s\r\n\r\n%s"
           %(from_address, to_address, subject, text))
    server.sendmail(from_address, to_address.split(','), msg)
    server.close()
