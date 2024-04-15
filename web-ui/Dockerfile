FROM python:3.8-slim

WORKDIR /app

COPY requirements.txt /app
RUN pip install -r requirements.txt

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
#CMD ["python", "app.py"]
