FROM python:3.12-slim
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
WORKDIR /app
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt \
    && useradd --create-home --uid 10001 kenny \
    && mkdir -p /app/data /app/config \
    && chown -R kenny:kenny /app/data /app/config
COPY app ./app
COPY examples ./examples
USER kenny
EXPOSE 8000
CMD ["python", "-m", "app"]
