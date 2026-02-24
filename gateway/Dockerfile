FROM m.daocloud.io/docker.io/library/node:22-slim

WORKDIR /srv/gateway

COPY gateway/package.json ./
RUN npm install

COPY gateway/ ./
COPY frontend/ /srv/frontend/

ENV PORT=8000
ENV PYTHON_EXEC_BASE=http://backend:8000

EXPOSE 8000

CMD ["npm", "run", "start"]
