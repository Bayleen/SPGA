FROM python:3.12-slim
WORKDIR /app
COPY requirements.txt /app/
RUN pip install --no-cache-dir -r requirements.txt
COPY spga /app/spga
COPY configs /app/configs
COPY app.py /app/app.py
ENV PYTHONUNBUFFERED=1
EXPOSE 8080
CMD ["sh", "-c", "uvicorn app:api --host 0.0.0.0 --port ${PORT:-8080} --workers 1"]